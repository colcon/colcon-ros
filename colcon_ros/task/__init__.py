# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os


def append_app_to_cpp(env):
    """Append AMENT_PREFIX_PATH to CMAKE_PREFIX_PATH."""
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH')
    if ament_prefix_path:
        cmake_prefix_path = env.get('CMAKE_PREFIX_PATH')
        cpp = cmake_prefix_path.split(os.pathsep) if cmake_prefix_path else []
        app = ament_prefix_path.split(os.pathsep)
        for p in app:
            if p not in cpp:
                cpp.append(p)
        env['CMAKE_PREFIX_PATH'] = os.pathsep.join(cpp)
