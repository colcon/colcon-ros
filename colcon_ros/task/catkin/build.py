# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from collections import OrderedDict
import os
from pathlib import Path

from colcon_cmake.task.cmake.build import CmakeBuildTask
from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.shell import get_shell_extensions
from colcon_core.task import TaskExtensionPoint
from colcon_ros.task.catkin import create_pythonpath_environment_hook
from colcon_ros.task.cmake import create_pkg_config_path_environment_hooks

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
            'Arguments matching other options must be prefixed by a space')
        parser.add_argument(
            '--catkin-skip-building-tests',
            action='store_true',
            help="By default the 'tests' target of 'catkin' packages is "
            "invoked. If running 'colcon test' later isn't intended this can "
            'be skipped')

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

        # invoke the build
        additional_targets = []
        # if no specific target is specified consider building the 'tests'
        # target and continue if such a target doesn't exist
        if args.cmake_target is None:
            if not args.catkin_skip_building_tests:
                additional_targets.append('tests')
                args.cmake_target_skip_unavailable = True
        rc = await extension.build(
            skip_hook_creation=True, additional_targets=additional_targets)

        # for catkin packages add additional hooks after the package has
        # been built and installed depending on the installed files
        additional_hooks = create_environment_hook(
            'ros_package_path', Path(args.install_base), self.context.pkg.name,
            'ROS_PACKAGE_PATH', 'share', mode='prepend')
        additional_hooks += create_pythonpath_environment_hook(
            args.build_base, Path(args.install_base), self.context.pkg.name)
        additional_hooks += create_pkg_config_path_environment_hooks(
            Path(args.install_base), self.context.pkg.name)

        # register hooks created via catkin_add_env_hooks
        shell_extensions = get_shell_extensions()
        file_extensions = OrderedDict()
        for shell_extensions_same_prio in shell_extensions.values():
            for shell_extension in shell_extensions_same_prio.values():
                for file_extension in shell_extension.get_file_extensions():
                    file_extensions[file_extension] = shell_extension
        custom_hooks_path = Path(args.install_base) / \
            'share' / self.context.pkg.name / 'catkin_env_hook'
        for file_extension, shell_extension in file_extensions.items():
            file_extension_hooks = sorted(custom_hooks_path.glob(
                '*.{file_extension}'.format_map(locals())))
            if file_extension_hooks:
                try:
                    # try to set CATKIN_ENV_HOOK_WORKSPACE explicitly before
                    # sourcing these hooks
                    additional_hooks.append(
                        shell_extension.create_hook_set_value(
                            'catkin_env_hook_workspace',
                            Path(args.install_base), self.context.pkg.name,
                            'CATKIN_ENV_HOOK_WORKSPACE',
                            ''))
                except NotImplementedError:
                    # since not all shell extensions might implement this
                    pass
                additional_hooks += file_extension_hooks

        create_environment_scripts(
            self.context.pkg, args, additional_hooks=additional_hooks)

        # ensure that the install base has the marker file
        # identifying it as a catkin workspace
        marker = Path(args.install_base) / '.catkin'
        marker.touch(exist_ok=True)

        return rc
