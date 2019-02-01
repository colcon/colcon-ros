# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.logging import colcon_logger
from colcon_core.shell import create_environment_hook

logger = colcon_logger.getChild(__name__)


def create_pythonpath_environment_hook(basepath, pkg_name):
    """
    Create a hook script for each primary shell to prepend to the PYTHONPATH.

    :param Path basepath: The path of the prefix
    :param str pkg_name: The package name
    :returns: The relative paths to the created hook scripts
    :rtype: list
    """
    hooks = []
    # prepend Python 2 specific path to PYTHONPATH if it exists
    if os.environ.get('ROS_PYTHON_VERSION', '2') == '2':
        for subdirectory in ('dist-packages', 'site-packages'):
            python_path = basepath / 'lib' / 'python2.7' / subdirectory
            logger.log(1, "checking '%s'" % python_path)
            if python_path.exists():
                rel_python_path = python_path.relative_to(basepath)
                hooks += create_environment_hook(
                    'python2path', basepath, pkg_name,
                    'PYTHONPATH', str(rel_python_path), mode='prepend')
    return hooks
