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


def extend_cpp_with_app(args):
    """Extend CMAKE_PREFIX_PATH with AMENT_PREFIX_PATH."""
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH')
    if ament_prefix_path:
        ament_prefix_path = ament_prefix_path.replace(
            os.pathsep, ';')
        if args.cmake_args is None:
            args.cmake_args = []
        # check if the CMAKE_PREFIX_PATH is explicitly set
        prefix = '-DCMAKE_PREFIX_PATH='
        for i, value in reversed(list(enumerate(args.cmake_args))):
            if not value.startswith(prefix):
                continue
            # extend the last existing entry
            existing = value[len(prefix):]
            if existing:
                existing = ';' + existing
            args.cmake_args[i] = \
                '-DCMAKE_PREFIX_PATH={ament_prefix_path}{existing}' \
                .format_map(locals())
            break
        else:
            # otherwise extend the environment variable
            existing = os.environ.get('CMAKE_PREFIX_PATH', '')
            if existing:
                existing = ';' + existing.replace(
                    os.pathsep, ';')
            args.cmake_args.append(
                '-DCMAKE_PREFIX_PATH={ament_prefix_path}{existing}'
                .format_map(locals()))
