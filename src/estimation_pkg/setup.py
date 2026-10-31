from setuptools import find_packages, setup
import os, glob

package_name = 'estimation_pkg'

# Install only runnable bundles, not multi-GB offline prediction archives.
# --symlink-install links these files. The GUI also scans source models/ so newly
# dropped bundles can be selected without copying the experiment's NPZ files.
bundle_files = {}
for path in glob.glob(os.path.join('models', '**', 'hrm_bundle.pt'), recursive=True):
    bundle_files.setdefault(os.path.dirname(path), []).append(path)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.xml'))),
        (os.path.join('share', package_name), ['estimation_pkg/config.json']),
        (os.path.join('share', package_name), ['estimation_pkg/id_localization_config.json']),
        (os.path.join('share', package_name), ['estimation_pkg/config_ROI_ref.json']),
        # (os.path.join('share', package_name, 'config'), glob('config/*.perspective')),
        (os.path.join('share', package_name), ['config/rqt_perspective/rqt_setting.perspective']),
        # location of .h5 model is LSTM-force-estimation/src/estimation_pkg/ (not LSTM-force-estimation/model)
        # (os.path.join('share', package_name), ['model/lstm_model_legacy.h5']),
        # Legacy TensorFlow assets were retired; do not require absent files.
    ] + [(os.path.join('share', package_name, directory), paths)
         for directory, paths in sorted(bundle_files.items())],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daeyun',
    maintainer_email='bigyun9375@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'segment_angle_estimator = estimation_pkg.segment_angle_estimation:main',
          'external_force_estimator = estimation_pkg.external_force_estimation:main',
          'force_id_estimator = estimation_pkg.force_id_estimation:main',
        ],
    },
)
