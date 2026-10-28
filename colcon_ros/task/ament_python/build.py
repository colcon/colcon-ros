# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import inspect
import os
from pathlib import Path
import re
import sys

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.shell import get_command_environment
from colcon_core.task import create_file
from colcon_core.task import install
from colcon_core.task import TaskExtensionPoint
from colcon_core.task.python import get_data_files_mapping
from colcon_core.task.python import get_setup_data
from colcon_core.task.python.build import PythonBuildTask

logger = colcon_logger.getChild(__name__)


class AmentPythonBuildTask(TaskExtensionPoint):
    """Build ROS packages with the build type 'ament_python'."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self):  # noqa: D102
        args = self.context.args
        logger.info(
            f"Building ROS package in '{args.path}' with build type "
            "'ament_python'")

        # reuse Python build task with additional logic
        extension = PythonBuildTask()
        extension.set_context(context=self.context)

        # additional hooks
        additional_hooks = create_environment_hook(
            'ament_prefix_path', Path(args.install_base),
            self.context.pkg.name, 'AMENT_PREFIX_PATH', '', mode='prepend')

        setup_py = Path(args.path) / 'setup.py'
        pyproject_toml = Path(args.path) / 'pyproject.toml'
        if not setup_py.is_file() and pyproject_toml.is_file():
            # Modern PEP 517 / 621 Python standard project
            return await self._build_python_project(args, additional_hooks)

        # get options from the Python manifest
        try:
            env = await get_command_environment(
                'setup_py', args.build_base, self.context.dependencies)
        except RuntimeError as e:
            logger.error(str(e))
            return 1
        setup_py_data = get_setup_data(self.context.pkg, env)

        # check if the package index and manifest are being installed
        data_files = get_data_files_mapping(
            setup_py_data.get('data_files', []) or [])
        installs_package_index = False
        installs_package_manifest = False

        # check if package index and manifest are being installed
        for source, destination in data_files.items():
            if sys.platform == 'win32':
                destination = Path(destination).as_posix()
            # work around data files incorrectly defined as not relative
            if os.path.isabs(source):
                source = os.path.relpath(source, args.path)
            if (
                destination == 'share/ament_index/resource_index/packages/' +
                self.context.pkg.name
            ):
                installs_package_index = True
            elif (
                source == 'package.xml' and
                destination == f'share/{self.context.pkg.name}/package.xml'
            ):
                installs_package_manifest = True

        # warn about missing explicit installation
        # for now implicitly install the marker and the manifest
        if not installs_package_index:
            # TODO remove magic helper in the future
            logger.warn(
                f"Package '{self.context.pkg.name}' doesn't explicitly "
                'install a marker in the package index (colcon-ros currently '
                'does it implicitly but that fallback will be removed in the '
                'future)')
            # create package marker in ament resource index
            create_file(
                args,
                'share/ament_index/resource_index/packages/'
                f'{self.context.pkg.name}')
        if not installs_package_manifest:
            # TODO remove magic helper in the future
            logger.warn(
                f"Package '{self.context.pkg.name}' doesn't explicitly "
                "install the 'package.xml' file (colcon-ros currently does it "
                'implicitly but that fallback will be removed in the future)')
            # copy / symlink package manifest
            install(
                args, 'package.xml',
                f'share/{self.context.pkg.name}/package.xml')

        return await extension.build(additional_hooks=additional_hooks)

    async def _build_python_project(self, args, additional_hooks):
        try:
            from colcon_core.task.python_project.build import \
                PythonProjectBuildTask
        except ImportError:
            logger.error(
                f"Package '{self.context.pkg.name}' uses pyproject.toml "
                'but colcon-core does not support python_project')
            return 1

        extension = PythonProjectBuildTask()
        extension.set_context(context=self.context)

        build_kwargs = {
            'additional_hooks': additional_hooks,
        }
        if (
            'script_dir_override' in
            inspect.signature(extension.build).parameters
        ):
            build_kwargs['script_dir_override'] = (
                Path('lib') / self.context.pkg.name
            )

        rc = await extension.build(**build_kwargs)
        if rc:
            return rc

        install_base = Path(args.install_base)
        added_paths = []

        index_rel = (
            'share/ament_index/resource_index/packages/'
            f'{self.context.pkg.name}'
        )
        index_dst = install_base / index_rel
        if not index_dst.exists() and not index_dst.is_symlink():
            create_file(args, index_rel)
            added_paths.append(index_dst)

        manifest_rel = f'share/{self.context.pkg.name}/package.xml'
        manifest_dst = install_base / manifest_rel
        if not manifest_dst.exists() and not manifest_dst.is_symlink():
            install(args, 'package.xml', manifest_rel)
            added_paths.append(manifest_dst)

        _record_installed_files(self.context.pkg, install_base, added_paths)
        return rc


def _record_installed_files(pkg, install_base, paths):
    if not paths:
        return
    try:
        from colcon_core.python_install_path import get_python_install_path
        from colcon_core.python_project.distribution import \
            InstalledDistribution
    except ImportError:  # pragma: no cover
        return

    dist_name = pkg.name
    try:
        from colcon_core.python_project.spec import load_and_cache_spec
    except ImportError:  # pragma: no cover
        pass
    else:
        spec = load_and_cache_spec(pkg)
        dist_name = spec.get('project', {}).get('name') or dist_name

    lib_dirs = {
        Path(get_python_install_path(key, {
            'base': str(install_base),
            'platbase': str(install_base),
        }))
        for key in ('purelib', 'platlib')
    }
    target_norm = re.sub(r'[-_.]+', '-', dist_name).lower()
    distributions = [
        d for d in InstalledDistribution.discover(
            name=dist_name,
            path=[str(d) for d in sorted(lib_dirs)],
            prefix_path=install_base,
        )
        if re.sub(r'[-_.]+', '-', d.name or '').lower() == target_norm
    ]
    for dist in distributions:
        record_path = dist.path / 'RECORD'
        if not record_path.is_file():
            continue
        lib_dir = dist.path.parent
        record_map = {}
        with record_path.open('r') as f:
            for line in f:
                stripped = line.rstrip('\r\n')
                if ',' in stripped:
                    parts = stripped.split(',', 2)
                    record_map[parts[0]] = tuple(parts)
        for path in paths:
            rel = Path(os.path.relpath(path, lib_dir)).as_posix()
            record_map[rel] = (rel, '', '')
        with record_path.open('w') as f:
            f.writelines(','.join(rec) + '\n' for rec in record_map.values())
