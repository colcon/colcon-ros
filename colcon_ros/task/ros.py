# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from pathlib import Path

from colcon_cmake.task.cmake.build import CmakeBuildTask
from colcon_cmake.task.cmake.test import CmakeTestTask
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
            help="Arbitrary arguments which are passed to all 'ament_cmake' "
            'packages (args which start with a dash must be prefixed with an '
            'escaped space `\ `, e.g.: `--ament-cmake-args \ -Dvar=val`)')
        parser.add_argument(
            '--catkin-cmake-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Arbitrary arguments which are passed to all 'catkin' "
            'packages (args which start with a dash must be prefixed with an '
            'escaped space `\ `, e.g.: `--catkin-cmake-args \ -Dvar=val`)')

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
            if args.catkin_cmake_args:
                if args.cmake_args is None:
                    args.cmake_args = []
                args.cmake_args += args.catkin_cmake_args
            # create hooks to (un)set CATKIN_SETUP_UTIL_ARGS environment
            # variable to mimic passing --extend along in plain shells
            create_file(
                args, 'share/{pkg.name}/hook/set_catkin_setup_util_args.sh'
                .format_map(locals()),
                content='CATKIN_SETUP_UTIL_ARGS=--extend')
            create_file(
                args, 'share/{pkg.name}/hook/unset_catkin_setup_util_args.sh'
                .format_map(locals()),
                content='unset CATKIN_SETUP_UTIL_ARGS')
            additional_hooks = [
                ['setup.bash', '--extend'],
                ['setup.bat'],
                ['share/{pkg.name}/hook/set_catkin_setup_util_args.sh'
                 .format_map(locals())],
                ['setup.sh', "# --extend doesn't work in a plain shell"],
                ['share/{pkg.name}/hook/unset_catkin_setup_util_args.sh'
                 .format_map(locals())]]

        elif build_type == 'cmake':
            extension = CmakeBuildTask()

        else:
            assert False, 'Unknown build type: ' + build_type

        extension.set_context(context=self.context)
        return await extension.build(additional_hooks=additional_hooks)

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
