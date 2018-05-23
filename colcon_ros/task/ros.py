# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path

from colcon_cmake.task.cmake.build import CmakeBuildTask
from colcon_cmake.task.cmake.test import CmakeTestTask
from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.task import create_file
from colcon_core.task import install
from colcon_core.task import TaskExtensionPoint
from colcon_core.task.python.build import PythonBuildTask
from colcon_core.task.python.test import PythonTestTask
from colcon_ros.package_identification.ros import get_package_with_build_type

logger = colcon_logger.getChild(__name__)


class RosTask(TaskExtensionPoint):
    """Build / test ROS packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--ament-cmake-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to all 'ament_cmake' packages. Every arg "
            'starting with a dash must be prefixed by a space,\n'
            'e.g. --ament-cmake-args " -Dvar=val"')
        parser.add_argument(
            '--catkin-cmake-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to all 'catkin' packages. Every arg starting "
            'with a dash must be prefixed by a space,\n'
            'e.g. --catkin-cmake-args " -Dvar=val"')

    async def build(self):  # noqa: D102
        args = self.context.args

        # get build type of package from manifest
        pkg, build_type = get_package_with_build_type(args.path)
        logger.info(
            "Building ROS package in '{args.path}' with build type "
            "'{build_type}'".format_map(locals()))

        # choose the task extension and additional hooks and arguments
        additional_hooks = []
        if build_type == 'ament_cmake':
            extension = CmakeBuildTask()
            additional_hooks += [
                'share/{pkg.name}/local_setup.bash'.format_map(locals()),
                'share/{pkg.name}/local_setup.bat'.format_map(locals()),
                'share/{pkg.name}/local_setup.ps1'.format_map(locals()),
                'share/{pkg.name}/local_setup.sh'.format_map(locals()),
            ]
            if args.symlink_install:
                if args.cmake_args is None:
                    args.cmake_args = []
                args.cmake_args.append('-DAMENT_CMAKE_SYMLINK_INSTALL=1')
            if args.ament_cmake_args:
                if args.cmake_args is None:
                    args.cmake_args = []
                args.cmake_args += args.ament_cmake_args

        elif build_type == 'ament_python':
            extension = PythonBuildTask()
            additional_hooks += create_environment_hook(
                'ament_prefix_path', Path(args.install_base), pkg.name,
                'AMENT_PREFIX_PATH', '', mode='prepend')
            # create package marker in ament resource index
            create_file(
                args, 'share/ament_index/resource_index/packages/{pkg.name}'
                .format_map(locals()))
            # copy / symlink package manifest
            install(
                args, 'package.xml', 'share/{pkg.name}/package.xml'
                .format_map(locals()))

        elif build_type == 'catkin':
            extension = CmakeBuildTask()
            if args.cmake_args is None:
                args.cmake_args = []
            args.cmake_args += ['-DCATKIN_BUILD_BINARY_PACKAGE=1']
            if args.catkin_cmake_args:
                args.cmake_args += args.catkin_cmake_args

        elif build_type == 'cmake':
            extension = CmakeBuildTask()

        else:
            assert False, 'Unknown build type: ' + build_type

        extension.set_context(context=self.context)
        if build_type != 'catkin':
            return await extension.build(additional_hooks=additional_hooks)
        else:
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
                            'python2path', Path(args.install_base), pkg.name,
                            'PYTHONPATH', str(rel_python_path), mode='prepend')
            create_environment_scripts(
                self.context.pkg, args, additional_hooks=additional_hooks)

            # ensure that the install base has the marker file
            # identifying it as a catkin workspace
            marker = Path(args.install_base) / '.catkin'
            marker.touch(exist_ok=True)

            return rc

    async def test(self):  # noqa: D102
        args = self.context.args

        # get build type of package from manifest
        pkg, build_type = get_package_with_build_type(args.path)
        logger.info(
            "Testing ROS package in '{args.path}' with build type "
            "'{build_type}'".format_map(locals()))

        # choose the task extension
        if build_type in ('ament_cmake', 'catkin', 'cmake'):
            extension = CmakeTestTask()
        elif build_type == 'ament_python':
            extension = PythonTestTask()
        else:
            assert False, 'Unknown build type: ' + build_type

        extension.set_context(context=self.context)
        return await extension.test()
