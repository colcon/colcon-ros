# Copyright 2026 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

"""Tests for AmentPythonBuildTask supporting modern PEP 621 packages."""

import asyncio
from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace

from colcon_core import plugin_system
from colcon_core.extension_point import clear_entry_point_cache
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.python_install_path import get_python_install_path
from colcon_core.task import TaskContext
from colcon_core.task.python import get_setup_data
from colcon_core.task.python.test import has_test_dependency
from colcon_ros.package_augmentation.ros_ament_python import \
    RosAmentPythonPackageAugmentation
from colcon_ros.task.ament_python.build import AmentPythonBuildTask
import pytest


@pytest.fixture(autouse=True)
def isolate_extension_cache_and_event_queue(monkeypatch):
    clear_entry_point_cache()
    plugin_system._extension_instances.clear()
    monkeypatch.setattr(
        TaskContext, 'put_event_into_queue', lambda self, event: None)
    yield
    clear_entry_point_cache()
    plugin_system._extension_instances.clear()


def _write_ros_pyproject_package(pkg_dir):
    pkg_dir.mkdir(parents=True, exist_ok=True)
    (pkg_dir / 'package.xml').write_text(
        '<?xml version="1.0"?>\n'
        '<package format="3">\n'
        '  <name>my_ros_pkg</name>\n'
        '  <version>0.1.0</version>\n'
        '  <description>Sample ament_python package</description>\n'
        '  <maintainer email="user@example.com">User</maintainer>\n'
        '  <license>Apache-2.0</license>\n'
        '  <export>\n'
        '    <build_type>ament_python</build_type>\n'
        '  </export>\n'
        '</package>\n')
    (pkg_dir / 'pyproject.toml').write_text(
        '[build-system]\n'
        'requires = ["flit_core >=3.2,<4"]\n'
        'build-backend = "flit_core.buildapi"\n\n'
        '[project]\n'
        'name = "my_ros_pkg"\n'
        'version = "0.1.0"\n'
        'description = "Sample ament_python package"\n'
        'readme = "README.md"\n'
        'dependencies = ["PyYAML"]\n\n'
        '[project.optional-dependencies]\n'
        'test = ["pytest"]\n\n'
        '[project.scripts]\n'
        'sample-node = "my_ros_pkg:main"\n')
    (pkg_dir / 'README.md').write_text('# my_ros_pkg\n')
    mod_dir = pkg_dir / 'my_ros_pkg'
    mod_dir.mkdir(exist_ok=True)
    (mod_dir / '__init__.py').write_text(
        '__version__ = "0.1.0"\n\n\n'
        'def main():\n'
        '    return 0\n')


def test_ament_python_build_pep621(tmp_path):
    """Test AmentPythonBuildTask end-to-end with a pyproject.toml package."""
    pytest.importorskip('colcon_core.task.python_project.build')
    pytest.importorskip('flit_core')

    pkg_dir = tmp_path / 'src' / 'my_ros_pkg'
    _write_ros_pyproject_package(pkg_dir)

    desc = PackageDescriptor(pkg_dir)
    desc.name = 'my_ros_pkg'
    desc.type = 'ros.ament_python'

    RosAmentPythonPackageAugmentation().augment_package(desc)
    assert has_test_dependency(get_setup_data(desc, {}), 'pytest')

    build_base = tmp_path / 'build'
    install_base = tmp_path / 'install'
    build_base.mkdir(parents=True)
    install_base.mkdir(parents=True)

    # Run build twice (clearing in-process ScriptMaker cache between runs to
    # mirror separate CLI invocations) to verify repeated builds preserve
    # ament_index and package.xml after remove_distributions runs.
    from colcon_core.python_project.wheel import _get_script_maker
    for _ in range(2):
        _get_script_maker.cache_clear()
        task = AmentPythonBuildTask()
        context = TaskContext(
            pkg=desc,
            args=SimpleNamespace(
                path=str(pkg_dir),
                build_base=str(build_base),
                install_base=str(install_base),
                symlink_install=False,
            ),
            dependencies={},
        )
        task.set_context(context=context)
        res = asyncio.run(task.build())
        assert res is None, f'Task build failed with: {res}'

    marker = (
        install_base / 'share' / 'ament_index' / 'resource_index' /
        'packages' / 'my_ros_pkg'
    )
    assert marker.is_file(), f'Missing ament_index marker at {marker}'

    pkg_xml = install_base / 'share' / 'my_ros_pkg' / 'package.xml'
    assert pkg_xml.is_file(), f'Missing package.xml at {pkg_xml}'

    dist_info = list(install_base.glob('**/my_ros_pkg-0.1.0.dist-info'))
    assert len(dist_info) == 1
    lib_dir = dist_info[0].parent
    assert (lib_dir / 'my_ros_pkg' / '__init__.py').is_file()

    # Verify [project.scripts] installed to ROS 2 lib/<pkg>/ directory
    node_dir = install_base / 'lib' / 'my_ros_pkg'
    assert (
        (node_dir / 'sample-node').is_file() or
        (node_dir / 'sample-node.exe').is_file()
    )
    assert not (install_base / 'bin' / 'sample-node').exists()

    # Verify ament_prefix_path hook and package-level scripts
    hook = (
        install_base / 'share' / 'my_ros_pkg' / 'hook' /
        'ament_prefix_path.dsv'
    )
    assert hook.is_file(), f'Missing ament_prefix_path hook at {hook}'
    pkg_dsv = install_base / 'share' / 'my_ros_pkg' / 'package.dsv'
    assert pkg_dsv.is_file()
    assert 'ament_prefix_path' in pkg_dsv.read_text()
    if sys.platform != 'win32':
        pkg_sh = install_base / 'share' / 'my_ros_pkg' / 'package.sh'
        assert pkg_sh.is_file()
        assert 'ament_prefix_path.sh' in pkg_sh.read_text()

    # Verify marker and package.xml are tracked once in RECORD
    record_lines = [
        line.split(',', 1)[0]
        for line in (dist_info[0] / 'RECORD').read_text().splitlines()
        if line.strip()
    ]
    assert len(record_lines) == len(set(record_lines))
    assert any(
        p.endswith('share/ament_index/resource_index/packages/my_ros_pkg')
        for p in record_lines
    )
    assert any(
        p.endswith('share/my_ros_pkg/package.xml')
        for p in record_lines
    )


def test_ament_python_build_python_project_unit(tmp_path, monkeypatch):
    """Unit test AmentPythonBuildTask delegation without optional tools."""
    pkg_dir = tmp_path / 'src' / 'my_ros_pkg'
    _write_ros_pyproject_package(pkg_dir)

    desc = PackageDescriptor(pkg_dir)
    desc.name = 'my_ros_pkg'
    desc.type = 'ros.ament_python'

    build_base = tmp_path / 'build'
    install_base = tmp_path / 'install'
    build_base.mkdir(parents=True)
    install_base.mkdir(parents=True)

    def _make_task():
        task = AmentPythonBuildTask()
        task.set_context(context=TaskContext(
            pkg=desc,
            args=SimpleNamespace(
                path=str(pkg_dir),
                build_base=str(build_base),
                install_base=str(install_base),
                symlink_install=False,
            ),
            dependencies={},
        ))
        return task

    # 1. Missing colcon_core.task.python_project.build returns 1 cleanly
    monkeypatch.setitem(
        sys.modules, 'colcon_core.task.python_project', None)
    monkeypatch.setitem(
        sys.modules, 'colcon_core.task.python_project.build', None)
    assert asyncio.run(_make_task().build()) == 1

    # 2. Stub PythonProjectBuildTask verifies flat hooks, script_dir_override,
    #    early failure return, and RECORD updates
    captured = {}
    state = {'rc': 2}

    class _StubBuildTask:

        def set_context(self, *, context):
            captured['context'] = context

        async def build(
            self, *, additional_hooks=None, script_dir_override=None
        ):
            captured['additional_hooks'] = additional_hooks
            captured['script_dir_override'] = script_dir_override
            if state['rc']:
                return state['rc']
            purelib = Path(get_python_install_path('purelib', {
                'base': str(install_base),
                'platbase': str(install_base),
            }))
            dist_info = purelib / 'my_ros_pkg-0.1.0.dist-info'
            dist_info.mkdir(parents=True, exist_ok=True)
            (dist_info / 'METADATA').write_text(
                'Metadata-Version: 2.1\nName: my-ros-pkg\nVersion: 0.1.0\n')
            (dist_info / 'RECORD').write_text(
                'my_ros_pkg/__init__.py,,\n')
            return None

    pkg_mod = ModuleType('colcon_core.task.python_project')
    build_mod = ModuleType('colcon_core.task.python_project.build')
    build_mod.PythonProjectBuildTask = _StubBuildTask
    monkeypatch.setitem(
        sys.modules, 'colcon_core.task.python_project', pkg_mod)
    monkeypatch.setitem(
        sys.modules, 'colcon_core.task.python_project.build', build_mod)

    # Non-zero build return code exits before installing marker/manifest
    assert asyncio.run(_make_task().build()) == 2
    marker = (
        install_base / 'share' / 'ament_index' / 'resource_index' /
        'packages' / 'my_ros_pkg'
    )
    pkg_xml = install_base / 'share' / 'my_ros_pkg' / 'package.xml'
    assert not marker.exists()
    assert not pkg_xml.exists()

    # Successful build installs marker + manifest and records them
    state['rc'] = None
    assert asyncio.run(_make_task().build()) is None
    assert captured['script_dir_override'] == Path('lib') / 'my_ros_pkg'
    assert isinstance(captured['additional_hooks'], list)
    assert all(
        not isinstance(item, list) for item in captured['additional_hooks']
    )
    assert marker.is_file()
    assert pkg_xml.is_file()

    # Second run when marker and manifest already exist preserves them
    marker.write_text('custom_index_content\n')
    assert asyncio.run(_make_task().build()) is None
    assert marker.read_text() == 'custom_index_content\n'


def test_ros_ament_python_augmentation_pyproject(tmp_path):
    """Test RosAmentPythonPackageAugmentation on pyproject.toml packages."""
    aug = RosAmentPythonPackageAugmentation()

    # Non-ament_python package ignored
    other = PackageDescriptor(tmp_path)
    other.type = 'ros.ament_cmake'
    aug.augment_package(other)
    assert 'get_python_setup_options' not in other.metadata

    # Missing both setup.py and pyproject.toml ignored
    empty_dir = tmp_path / 'empty_pkg'
    empty_dir.mkdir()
    empty_desc = PackageDescriptor(empty_dir)
    empty_desc.type = 'ros.ament_python'
    aug.augment_package(empty_desc)
    assert 'get_python_setup_options' not in empty_desc.metadata

    # pyproject.toml + setup.cfg + PEP 735 dependency-groups
    pkg_dir = tmp_path / 'modern_ros_pkg'
    _write_ros_pyproject_package(pkg_dir)
    (pkg_dir / 'pyproject.toml').write_text(
        (pkg_dir / 'pyproject.toml').read_text() +
        '\n[dependency-groups]\n'
        'test = ["pytest-cov"]\n')
    (pkg_dir / 'setup.cfg').write_text(
        '[options]\n'
        'zip_safe = false\n')

    desc = PackageDescriptor(pkg_dir)
    desc.name = 'my_ros_pkg'
    desc.type = 'ros.ament_python'
    aug.augment_package(desc)

    options = get_setup_data(desc, {})
    assert options['install_requires'] == ['PyYAML']
    assert options['extras_require']['test'] == ['pytest']
    assert 'pytest-cov' in options['tests_require']
    assert has_test_dependency(options, 'pytest')
    assert has_test_dependency(options, 'pytest-cov')
