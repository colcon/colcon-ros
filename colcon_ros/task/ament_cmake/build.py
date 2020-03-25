# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os

from colcon_cmake.task.cmake.build import CmakeBuildTask
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import get_shell_extensions
from colcon_core.task import TaskExtensionPoint
from colcon_ros.task import add_app_to_cpp

logger = colcon_logger.getChild(__name__)


class AmentCmakeBuildTask(TaskExtensionPoint):
    """Build ROS packages with the build type 'ament_cmake'."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--ament-cmake-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to 'ament_cmake' packages. "
            'Arguments matching other options must be prefixed by a space')

    async def build(self):  # noqa: D102
        args = self.context.args
        logger.info(
            "Building ROS package in '{args.path}' with build type "
            "'ament_cmake'".format_map(locals()))

        # reuse CMake build task with additional logic
        extension = CmakeBuildTask()
        extension.set_context(context=self.context)

        # add a hook for each available shell
        additional_hooks = []
        shell_extensions = get_shell_extensions()
        file_extensions = []
        for shell_extensions_same_prio in shell_extensions.values():
            for shell_extension in shell_extensions_same_prio.values():
                file_extensions += shell_extension.get_file_extensions()
        for file_extension in sorted(file_extensions):
            additional_hooks.append(
                'share/{self.context.pkg.name}/local_setup.{file_extension}'
                .format_map(locals()))

        # additional arguments
        if args.test_result_base:
            if args.cmake_args is None:
                args.cmake_args = []
            # ament_cmake appends the project name itself
            args.cmake_args.append(
                '-DAMENT_TEST_RESULTS_DIR=' +
                os.path.dirname(args.test_result_base))
        if args.symlink_install:
            if args.cmake_args is None:
                args.cmake_args = []
            args.cmake_args.append('-DAMENT_CMAKE_SYMLINK_INSTALL=1')
        if args.ament_cmake_args:
            if args.cmake_args is None:
                args.cmake_args = []
            args.cmake_args += args.ament_cmake_args

        return await extension.build(
            environment_callback=add_app_to_cpp,
            additional_hooks=additional_hooks)
