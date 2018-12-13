# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from unittest.mock import Mock

from colcon_ros.package_identification.ros import _create_metadata


def test_version_contraint_eq():
    mock = Mock()
    mock.version_gt = None
    mock.version_lt = None
    mock.version_lte = None
    mock.version_gte = None

    mock.version_eq = '1.1.1'

    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 1
    assert metadata['version_eq'] == '1.1.1'


def test_version_contraint_gt():
    mock = Mock()
    mock.version_eq = None
    mock.version_lt = None
    mock.version_lte = None
    mock.version_gte = None

    mock.version_gt = '1.1.2'

    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 1
    assert metadata['version_gt'] == '1.1.2'


def test_version_contraint_lt():
    mock = Mock()
    mock.version_eq = None
    mock.version_gt = None
    mock.version_lte = None
    mock.version_gte = None

    mock.version_lt = '1.1.5'

    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 1
    assert metadata['version_lt'] == '1.1.5'


def test_version_contraint_lte():
    mock = Mock()
    mock.version_eq = None
    mock.version_gt = None
    mock.version_lt = None
    mock.version_gte = None

    mock.version_lte = '10.1.3'

    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 1
    assert metadata['version_lte'] == '10.1.3'


def test_version_contraint_gte():
    mock = Mock()
    mock.version_eq = None
    mock.version_gt = None
    mock.version_lt = None
    mock.version_lte = None

    mock.version_gte = '10.45.5'

    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 1
    assert metadata['version_gte'] == '10.45.5'


def test_version_constraint_none():
    mock = Mock()
    mock.version_eq = None
    mock.version_gt = None
    mock.version_lt = None
    mock.version_lte = None
    mock.version_gte = None
    metadata = _create_metadata(mock)
    assert len(metadata.keys()) == 0
