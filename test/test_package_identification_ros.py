# Copyright 2016-2018 Dirk Thomas
# Copyright 2024 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

from pathlib import Path
from tempfile import TemporaryDirectory

from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.package_identification import IgnoreLocationException
from colcon_ros.package_identification.ros import _cached_packages
from colcon_ros.package_identification.ros import RosPackageIdentification
import pytest


def test_identify():
    extension = RosPackageIdentification()
    augmentation_extension = extension

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        desc = PackageDescriptor(basepath)
        desc.type = 'other'
        assert extension.identify(desc) is None
        assert desc.name is None

        _cached_packages.clear()
        desc.type = None
        assert extension.identify(desc) is None
        assert desc.name is None
        assert desc.type is None

        _cached_packages.clear()
        basepath = Path(basepath)
        for marker in ('AMENT_IGNORE', 'CATKIN_IGNORE'):
            marker_path = basepath / marker
            marker_path.touch()
            with pytest.raises(IgnoreLocationException):
                extension.identify(desc)
            marker_path.unlink()

        _cached_packages.clear()
        (basepath / 'package.xml').write_text(
            '<package format="3">\n'
            '  <name>pkg-name</name>\n'
            '  <version>0.0.0</version>\n'
            '  <description>test package</description>\n'
            '  <maintainer email="foobar@example.com">Foo Bar</maintainer>\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n')
        assert extension.identify(desc) is None
        assert desc.name == 'pkg-name'
        assert desc.type == 'ros.catkin'
        assert not desc.dependencies
        assert not desc.metadata

        augmentation_extension.augment_packages([desc])
        assert desc.metadata['version'] == '0.0.0'
        assert set(desc.dependencies.keys()) == {'build', 'run', 'test'}
        assert not desc.dependencies['build']
        assert not desc.dependencies['run']
        assert not desc.dependencies['test']

        _cached_packages.clear()
        desc = PackageDescriptor(basepath)
        (basepath / 'package.xml').write_text(
            '<package format="3">\n'
            '  <name>other-name</name>\n'
            '  <version>0.0.0</version>\n'
            '  <description>test package</description>\n'
            '  <maintainer email="foobar@example.com">Foo Bar</maintainer>\n'
            '  <license>Apache-2.0</license>\n'
            '  <buildtool_depend>build</buildtool_depend>\n'
            '  <buildtool_export_depend version_gt="1.2.3">'
            'runA</buildtool_export_depend>\n'
            '  <test_depend>test</test_depend>\n'
            '  <exec_depend>runB</exec_depend>\n'
            '  <export>\n'
            '    <build_type>ament_cmake</build_type>\n'
            '  </export>\n'
            '</package>\n')
        assert extension.identify(desc) is None
        assert desc.name == 'other-name'
        assert desc.type == 'ros.ament_cmake'
        assert not desc.dependencies
        assert not desc.metadata

        augmentation_extension.augment_packages([desc])
        assert desc.metadata['version'] == '0.0.0'
        assert set(desc.dependencies.keys()) == {'build', 'run', 'test'}
        assert desc.dependencies['build'] == {'build'}
        assert desc.dependencies['run'] == {'runA', 'runB'}
        dep = next(x for x in desc.dependencies['run'] if x == 'runA')
        assert dep.metadata['version_gt'] == '1.2.3'
        assert desc.dependencies['test'] == {'test'}

        assert desc.metadata['maintainers'] == ['Foo Bar <foobar@example.com>']

        _cached_packages.clear()
        desc = PackageDescriptor(basepath)
        (basepath / 'manifest.xml').touch()
        assert extension.identify(desc) is None
        assert desc.type == 'ros.ament_cmake'

        _cached_packages.clear()
        desc = PackageDescriptor(basepath)
        (basepath / 'package.xml').unlink()
        with pytest.raises(IgnoreLocationException):
            extension.identify(desc)
        (basepath / 'manifest.xml').unlink()
