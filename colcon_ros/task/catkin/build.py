# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path

from colcon_cmake.task.cmake.build import CmakeBuildTask
from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.shell import get_shell_extensions
from colcon_core.task import TaskExtensionPoint

logger = colcon_logger.getChild(__name__)


class CatkinBuildTask(TaskExtensionPoint):
    """Build ROS packages with the build type 'catkin'."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--catkin-cmake-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to 'catkin' packages. "
            'Arguments matching other options must be prefixed by a space,\n'
            'e.g. --catkin-cmake-args " --help"')

    async def build(self):  # noqa: D102
        args = self.context.args
        logger.info(
            "Building ROS package in '{args.path}' with build type 'catkin'"
            .format_map(locals()))

        # reuse CMake build task with additional logic
        extension = CmakeBuildTask()
        extension.set_context(context=self.context)

        # additional arguments
        if args.cmake_args is None:
            args.cmake_args = []
        args.cmake_args += ['-DCATKIN_INSTALL_INTO_PREFIX_ROOT=0']
        if args.test_result_base:
            # catkin appends the project name itself
            args.cmake_args.append(
                '-DCATKIN_TEST_RESULTS_DIR=' +
                os.path.dirname(args.test_result_base))
        if args.catkin_cmake_args:
            args.cmake_args += args.catkin_cmake_args

        # additional hooks
        additional_hooks = create_environment_hook(
            'ros_package_path', Path(args.install_base), self.context.pkg.name,
            'ROS_PACKAGE_PATH', 'share', mode='prepend')

        # for catkin packages add additional hooks after the package has
        # been built and installed depending on the installed files
        rc = await extension.build(skip_hook_creation=True)

        # add Python 2 specific path to PYTHONPATH if it exists
        if os.environ.get('ROS_PYTHON_VERSION', '2') == '2':
            for subdirectory in ('dist-packages', 'site-packages'):
                python_path = Path(args.install_base) / \
                    'lib' / 'python2.7' / subdirectory
                logger.log(1, "checking '%s'" % python_path)
                if python_path.exists():
                    rel_python_path = python_path.relative_to(
                        args.install_base)
                    additional_hooks += create_environment_hook(
                        'python2path', Path(args.install_base),
                        self.context.pkg.name,
                        'PYTHONPATH', str(rel_python_path), mode='prepend')

        # register hooks created via catkin_add_env_hooks
        shell_extensions = get_shell_extensions()
        file_extensions = []
        for shell_extensions_same_prio in shell_extensions.values():
            for shell_extension in shell_extensions_same_prio.values():
                file_extensions += shell_extension.get_file_extensions()
        custom_hooks_path = Path(args.install_base) / \
            'share' / self.context.pkg.name / 'catkin_env_hook'
        for file_extension in file_extensions:
            additional_hooks += sorted(
                custom_hooks_path.glob(
                    '*.{file_extension}'.format_map(locals())))

        create_environment_scripts(
            self.context.pkg, args, additional_hooks=additional_hooks)

        # ensure that the install base has the marker file
        # identifying it as a catkin workspace
        marker = Path(args.install_base) / '.catkin'
        marker.touch(exist_ok=True)

        return rc
