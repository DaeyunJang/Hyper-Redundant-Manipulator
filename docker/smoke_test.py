#!/usr/bin/env python3
# AUTHOR: Daeyun
"""Exercise installed software without ROS nodes, device access, or motor TCP.

Optional model checks use synthetic zero inputs on CPU. They verify checkpoint
loading and finite outputs, not physical force accuracy or live sensor timing.
"""

import argparse
import ctypes
import importlib
import os
from pathlib import Path
import sys
import tempfile


def check_imports():
    """Import the active runtime modules, without calling any node's main()."""
    modules = (
        'rclpy', 'rosbag2_py', 'cv2', 'cv_bridge', 'PyQt5.QtWidgets',
        'numpy', 'scipy', 'torch', 'serial', 'filterpy', 'threadpoolctl',
        'custom_interfaces.msg', 'realsense2_camera_msgs.msg', 'apriltag_msgs.msg',
        'estimation_pkg.postprocess', 'estimation_pkg.segment_angle_estimation',
        'estimation_pkg.force_id_estimation', 'estimation_pkg.force_id_runtime',
        'gui_py_pkg.gui_node', 'serial_pkg.serial_read', 'record_pkg.record_node',
    )
    for name in modules:
        importlib.import_module(name)
    import numpy as np
    import scipy
    import torch
    from scipy.integrate import quadrature  # noqa: F401
    from scipy.misc import derivative  # noqa: F401

    if int(np.__version__.split('.')[0]) >= 2:
        raise RuntimeError('ROS Humble cv_bridge requires the pinned NumPy 1.x ABI.')
    if scipy.__version__ != '1.13.1':
        raise RuntimeError(f'Expected SciPy 1.13.1, found {scipy.__version__}.')
    if torch.__version__ != '2.7.1+cpu':
        raise RuntimeError(f'Expected CPU torch 2.7.1+cpu, found {torch.__version__}.')
    print(f'PASS imports: Python {sys.version.split()[0]}, NumPy {np.__version__}, '
          f'SciPy {scipy.__version__}, Torch {torch.__version__}')


def check_packages():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    for name in (
        'custom_interfaces', 'image_rate_probe', 'estimation_pkg', 'gui_py_pkg',
        'serial_pkg', 'record_pkg', 'robot_control_pkg', 'tcp_pkg', 'launcher',
        'realsense2_camera', 'realsense2_camera_msgs', 'image_proc', 'apriltag_ros',
    ):
        get_package_prefix(name)
    try:
        get_package_prefix('daq_pkg')
    except PackageNotFoundError:
        pass
    else:
        raise RuntimeError('The unused native daq_pkg must not be installed.')
    # Inspect files only: starting tcpclient would attempt a motor connection.
    tcp_prefix = Path(get_package_prefix('tcp_pkg'))
    for name in ('tcpclient', 'demo_node'):
        executable = tcp_prefix / 'lib' / 'tcp_pkg' / name
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f'Missing installed TCP executable: {executable}')
    for name in ('_launch.py', '_launch_demo.py'):
        launch_file = tcp_prefix / 'share' / 'tcp_pkg' / 'launch' / name
        if not launch_file.is_file():
            raise RuntimeError(f'Missing installed TCP launch file: {launch_file}')
    camera = Path(get_package_prefix('realsense2_camera'))
    expected_camera = Path('/opt/hrm_ws/.cuda_realsense/install/realsense-ros')
    if camera.resolve() != expected_camera:
        raise RuntimeError(f'Wrong RealSense overlay selected: {camera}')
    library = Path('/opt/hrm_ws/.cuda_realsense/install/librealsense/lib/librealsense2.so')
    # Dynamic linking only: do not create an SDK context or enumerate cameras.
    ctypes.CDLL(str(library))
    print('PASS ROS package registration, TCP executables/launch files, and '
          'CUDA RealSense library linkage; TCP was not started')


def check_qt_and_images():
    import numpy as np
    from cv_bridge import CvBridge
    from PyQt5.QtWidgets import QApplication, QLabel

    app = QApplication.instance() or QApplication(['hrm-image-smoke'])
    label = QLabel('HRM container offscreen check')
    label.resize(240, 40)
    label.show()
    app.processEvents()
    if label.grab().isNull():
        raise RuntimeError('Qt could not render an offscreen widget.')
    label.close()
    bridge = CvBridge()
    pixels = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    message = bridge.cv2_to_imgmsg(pixels, encoding='bgr8')
    np.testing.assert_array_equal(bridge.imgmsg_to_cv2(message, 'bgr8'), pixels)
    print('PASS offscreen Qt rendering and cv_bridge image roundtrip')


def check_synthetic_inference():
    import numpy as np
    import torch
    from estimation_pkg.force_id_models import IndependentHRM
    from estimation_pkg.force_id_runtime import BundleRuntime

    torch.set_num_threads(1)
    # Known biases make the descaled output and ID unambiguous while exercising
    # strict weights-only loading, both neural branches, and separate scalers.
    bundle = {
        'schema_version': 2, 'kind': 'independent_hrm', 'mode': 'class0',
        'input_dim': 2, 'num_classes': 3, 'feature_columns': ['first', 'second'],
        'force_name': 'small_gru', 'location_name': 'transformer',
        'force_params': {}, 'location_params': {},
        'config': {'input_columns': ['first', 'second'], 'window_samples': 30,
                   'active_ids': [2, 18], 'force_frame': 'hrm_base'},
        'force_scope': 'tip', 'force_unit': 'N',
        'future_hrm_control_multiplier': -1,
        'scalers': {
            'force': {'x_mean': [2., 3.], 'x_std': [2., 4.],
                      'y_mean': [1., 2., 3.], 'y_std': [.1, .2, .3]},
            'location': {'x_mean': [-1., -2.], 'x_std': [3., 5.]},
        },
    }
    model = IndependentHRM(bundle)
    with torch.no_grad():
        for parameter in model.parameters():
            parameter.zero_()
        model.force_net.head.bias.copy_(torch.tensor([1., -2., 0.]))
        model.location_net.head.bias.copy_(torch.tensor([0., 1., 2.]))
    bundle['state_dict'] = model.state_dict()
    with tempfile.TemporaryDirectory(prefix='hrm-smoke-') as directory:
        path = Path(directory) / 'hrm_bundle.pt'
        torch.save(bundle, path)
        prediction = BundleRuntime(path, device='cpu').predict(np.zeros((30, 2)))
    np.testing.assert_allclose(prediction.force_N, [1.1, 1.6, 3.], atol=1e-6)
    np.testing.assert_allclose(prediction.probabilities.sum(), 1., atol=1e-6)
    if prediction.predicted_id != 18 or prediction.probabilities.shape != (19,):
        raise RuntimeError('Unexpected synthetic force/contact prediction.')
    print('PASS CPU force/contact checkpoint load and known-output inference')


def check_external_models(args):
    import numpy as np
    from estimation_pkg.force_id_runtime import BundleRuntime, discover_bundles

    paths = [Path(value).expanduser().resolve(strict=True) for value in args.model]
    if args.models_root:
        root = Path(args.models_root).expanduser().resolve(strict=True)
        paths.extend(discover_bundles(root))
    paths = sorted(set(paths))
    if args.require_models and not paths:
        raise RuntimeError('No models found; mount trusted bundles and supply --models-root.')
    for path in paths:
        runtime = BundleRuntime(path, device='cpu')
        prediction = runtime.predict(np.zeros((runtime.window_samples, runtime.input_dim)))
        np.testing.assert_allclose(prediction.probabilities.sum(), 1., atol=1e-5)
        print(f'PASS mounted checkpoint: {path} '
              f'({runtime.window_samples} samples, {runtime.input_dim} features)')
    if not paths:
        print('SKIP external checkpoints: use --model FILE or --models-root DIRECTORY '
              '--require-models to validate mounted bundles')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', action='append', default=[], metavar='FILE',
                        help='Trusted hrm_bundle.pt to validate on CPU; may be repeated')
    parser.add_argument('--models-root', metavar='DIRECTORY',
                        help='Recursively validate every hrm_bundle.pt under this directory')
    parser.add_argument('--require-models', action='store_true',
                        help='Fail unless an external checkpoint was found and validated')
    args = parser.parse_args()
    # Force offscreen checks even when invoked from a displayed GUI container.
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    os.environ['MPLBACKEND'] = 'Agg'
    check_imports()
    check_packages()
    check_qt_and_images()
    check_synthetic_inference()
    check_external_models(args)
    print('PASS software smoke test; no ROS nodes, hardware, or motor TCP were started')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'FAIL: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1) from error
