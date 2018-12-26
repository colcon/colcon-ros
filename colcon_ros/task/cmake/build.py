# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0


from colcon_cmake.task.cmake.build import CmakeBuildTask as CmakeBuildTask_
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint
from colcon_ros.task import append_app_to_cpp

logger = colcon_logger.getChild(__name__)


class CmakeBuildTask(TaskExtensionPoint):
    """Build ROS packages with the build type 'cmake'."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self):  # noqa: D102
        args = self.context.args
        logger.info(
            "Building ROS package in '{args.path}' with build type 'cmake'"
            .format_map(locals()))

        # reuse CMake build task with additional logic
        extension = CmakeBuildTask_()
        extension.set_context(context=self.context)

        return await extension.build(environment_callback=append_app_to_cpp)
