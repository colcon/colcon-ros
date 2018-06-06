# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_identification import IgnoreLocationException
from colcon_core.package_identification import logger
from colcon_core.package_identification \
    import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_core.plugin_system import SkipExtensionException
from colcon_python_setup_py.package_identification.python_setup_py \
    import get_setup_arguments_with_context


# mapping paths to tuples containing the ROS package and its build type
_cached_packages = {}


class RosPackageIdentification(
    PackageIdentificationExtensionPoint, PackageAugmentationExtensionPoint
):
    """Identify ROS packages with `package.xml` files."""

    # the priority needs to be higher than the extensions identifying packages
    # using the build systems supported by ROS (CMake and Python)
    PRIORITY = 150

    def __init__(self):  # noqa: D107
        satisfies_version(
            PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
        # check if the necessary dependency to parse the manifest is available
        try:
            import catkin_pkg  # noqa: F401
        except ImportError:
            raise SkipExtensionException(
                "The Python module 'catkin_pkg' could not be imported, "
                'therefore ROS packages can not be identified')

    def identify(self, desc):  # noqa: D102
        # ignore packages which have been identified with a different type
        if desc.type is not None and desc.type != 'ros':
            return

        # skip paths with an ignore marker file
        if (desc.path / 'CATKIN_IGNORE').exists():
            raise IgnoreLocationException()
        if (desc.path / 'AMENT_IGNORE').exists():
            raise IgnoreLocationException()

        # parse package manifest and get build type
        pkg, build_type = get_package_with_build_type(str(desc.path))
        if not pkg or not build_type:
            # if it is not a wet ROS package check for a dry ROS package
            if (desc.path / 'manifest.xml').exists():
                # ignore location to avoid being identified as a CMake package
                raise IgnoreLocationException()
            return

        desc.type = 'ros'

        # use package name from manifest if not already set
        # e.g. from external configuration
        if desc.name is None:
            desc.name = pkg.name

        # get dependencies
        for d in pkg.build_depends + pkg.buildtool_depends:
            assert d.evaluated_condition is not None
            if d.evaluated_condition:
                desc.dependencies['build'].add(d.name)

        for d in (
            pkg.build_export_depends +
            pkg.buildtool_export_depends +
            pkg.exec_depends
        ):
            assert d.evaluated_condition is not None
            if d.evaluated_condition:
                desc.dependencies['run'].add(d.name)

        for d in pkg.test_depends:
            assert d.evaluated_condition is not None
            if d.evaluated_condition:
                desc.dependencies['test'].add(d.name)

        def getter(env):
            nonlocal desc
            return get_setup_arguments_with_context(
                str(desc.path / 'setup.py'), env)

        desc.metadata['get_python_setup_options'] = getter

    def augment_packages(
        self, descs, *, additional_argument_names=None
    ):  # noqa: D102
        # get all parsed ROS package manifests
        global _cached_packages
        pkgs = {}
        for desc in descs:
            if str(desc.path) not in _cached_packages:
                continue
            pkg = _cached_packages[str(desc.path)][0]
            if pkg:
                pkgs[pkg] = desc

        # resolve group members and add them to the descriptor dependencies
        for pkg, desc in pkgs.items():
            for group_depend in pkg.group_depends:
                assert group_depend.evaluated_condition is not None
                if not group_depend.evaluated_condition:
                    continue
                group_depend.extract_group_members(pkgs)
                for name in group_depend.members:
                    desc.dependencies['build'].add(name)
                    desc.dependencies['run'].add(name)


def get_package_with_build_type(path: str):
    """Get the ROS package and its build type for the given path."""
    global _cached_packages
    if path not in _cached_packages:
        pkg = _get_package(path)
        build_type = _get_build_type(pkg) if pkg else None
        _cached_packages[path] = (pkg, build_type)
    return _cached_packages[path]


def _get_package(path: str):
    """Get the ROS package for the given path."""
    from catkin_pkg.package import InvalidPackage
    from catkin_pkg.package import package_exists_at
    from catkin_pkg.package import parse_package

    if not package_exists_at(path):
        return None

    try:
        pkg = parse_package(path)
    except (AssertionError, InvalidPackage):
        return None

    pkg.evaluate_conditions(os.environ)
    return pkg


def _get_build_type(pkg):
    """Get the build type of the ROS package."""
    from catkin_pkg.package import InvalidPackage
    try:
        build_type = pkg.get_build_type()
    except InvalidPackage:
        logger.warn(
            "ROS package '{pkg.name}' in '{path}' has more than one "
            'build type'.format_map(locals()))
        return None

    if build_type not in ('ament_cmake', 'ament_python', 'catkin', 'cmake'):
        path = os.path.dirname(pkg.filename)
        logger.debug(
            "ROS package '{pkg.name}' in '{path}' has unknown build "
            "type '{build_type}'".format_map(locals()))
        return None

    return build_type
