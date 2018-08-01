# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.task import create_file
from colcon_core.task import install
from colcon_core.task import TaskExtensionPoint
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
            "Building ROS package in '{args.path}' with build type "
            "'ament_python'".format_map(locals()))

        # reuse Python build task with additional logic
        extension = PythonBuildTask()
        extension.set_context(context=self.context)

        # additional hooks
        additional_hooks = create_environment_hook(
            'ament_prefix_path', Path(args.install_base),
            self.context.pkg.name, 'AMENT_PREFIX_PATH', '', mode='prepend')
        # create package marker in ament resource index
        create_file(
            args,
            'share/ament_index/resource_index/packages/{self.context.pkg.name}'
            .format_map(locals()))
        # copy / symlink package manifest
        install(
            args, 'package.xml', 'share/{self.context.pkg.name}/package.xml'
            .format_map(locals()))

        return await extension.build(additional_hooks=additional_hooks)
