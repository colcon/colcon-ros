# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_identification.python import get_configuration
from colcon_core.package_identification.python import is_reading_cfg_sufficient
from colcon_core.plugin_system import satisfies_version
from colcon_python_setup_py.package_identification.python_setup_py \
    import get_setup_information


class RosAmentPythonPackageAugmentation(PackageAugmentationExtensionPoint):
    """Augment ament_python packages with information from setup files."""

    # the priority needs to be higher than the extensions augmenting packages
    # using the setup.py/setup.cfg files
    PRIORITY = 150

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def augment_package(  # noqa: D102
        self, desc, *, additional_argument_names=None
    ):
        if desc.type != 'ros.ament_python':
            return

        if 'get_python_setup_options' in desc.metadata:
            return

        setup_py = desc.path / 'setup.py'
        setup_cfg = desc.path / 'setup.cfg'
        if not setup_py.is_file():
            pyproject_toml = desc.path / 'pyproject.toml'
            if not pyproject_toml.is_file():
                return
            options = _get_pyproject_options(desc, setup_cfg)
            if options is None:
                return

            def getter(env):
                return options

            desc.metadata['get_python_setup_options'] = getter
            return

        for _ in (1, ):
            # try to get information from setup.cfg file
            if setup_cfg.is_file():
                if is_reading_cfg_sufficient(setup_py):
                    config = get_configuration(setup_cfg)
                    name = config.get('metadata', {}).get('name')
                    if name:
                        options = config.get('options', {})

                        def getter(env):  # noqa: F811
                            return options
                        break
        else:
            # use information from setup.py file

            def getter(env):  # noqa: F811
                return get_setup_information(
                    desc.path / 'setup.py', env=env)

        desc.metadata['get_python_setup_options'] = getter


def _get_pyproject_options(desc, setup_cfg):
    try:
        from colcon_core.python_project.spec import load_and_cache_spec
    except ImportError:
        return None

    options = {}
    if setup_cfg.is_file():
        options.update(get_configuration(setup_cfg).get('options', {}))

    spec = load_and_cache_spec(desc)
    project = spec.get('project') or {}
    if project.get('dependencies'):
        options['install_requires'] = list(project['dependencies'])

    optional_deps = project.get('optional-dependencies') or {}
    extras = dict(options.get('extras_require') or {})
    for key, values in optional_deps.items():
        if isinstance(values, list):
            extras[key] = list(values)
    if extras:
        options['extras_require'] = extras

    dep_groups = spec.get('dependency-groups') or {}
    group_tests = [
        item
        for group in ('test', 'tests', 'testing')
        for item in (dep_groups.get(group) or ())
        if isinstance(item, str)
    ]
    if group_tests:
        existing_tests = list(options.get('tests_require') or ())
        options['tests_require'] = existing_tests + [
            item for item in group_tests if item not in existing_tests
        ]

    return options
