# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

from pathlib import Path
from tempfile import TemporaryDirectory

from colcon_core.package_descriptor import PackageDescriptor
from colcon_ros.package_augmentation.ros_ament_index \
    import RosAmentIndexPackageAugmentation


def test_ament_index_augmentation():
    with TemporaryDirectory() as mock_prefix:
        mock_prefix = Path(mock_prefix)

        index_path = mock_prefix / 'share' / 'ament_index' / 'resource_index'
        dep_index_path = index_path / 'package_run_dependencies'
        dep_index_path.mkdir(parents=True)
        (dep_index_path / 'pkg_no_deps').write_text('')
        (dep_index_path / 'pkg_with_deps').write_text('dep1;dep2')

        extension = RosAmentIndexPackageAugmentation()

        # Non-existing
        desc = PackageDescriptor(mock_prefix)
        desc.name = 'pkg_not_in_index'
        desc.type = 'installed'
        extension.augment_packages((desc,))
        assert 'installed' == desc.type
        assert not desc.get_dependencies(categories=('run',))

        # Wrong type
        desc = PackageDescriptor(mock_prefix)
        desc.name = 'pkg_with_deps'
        desc.type = 'installed.other_type'
        extension.augment_packages((desc,))
        assert 'installed.other_type' == desc.type
        assert not desc.get_dependencies(categories=('run',))

        # Existing with deps
        desc = PackageDescriptor(mock_prefix)
        desc.name = 'pkg_with_deps'
        desc.type = 'installed'
        extension.augment_packages((desc,))
        assert 'installed.ros.ament' == desc.type
        assert {'dep1', 'dep2'} == desc.get_dependencies(categories=('run',))

        # Existing without deps
        desc = PackageDescriptor(mock_prefix)
        desc.name = 'pkg_no_deps'
        desc.type = 'installed'
        extension.augment_packages((desc,))
        assert 'installed.ros.ament' == desc.type
        assert not desc.get_dependencies(categories=('run',))
