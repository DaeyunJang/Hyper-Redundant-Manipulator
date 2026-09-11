import os
import sys
import glob
from xml.sax.saxutils import escape

import cv2
import matplotlib.pyplot as plt
import time
from datetime import datetime
import json
from tqdm import tqdm
from natsort import natsorted
import line_profiler
import cProfile
from numba import jit

import cv2
import matplotlib.pyplot as plt
import numpy as np
from skimage.morphology import skeletonize
from fil_finder import FilFinder2D
import astropy.units as u
from scipy.optimize import curve_fit, brentq, minimize_scalar, root_scalar, OptimizeWarning
from scipy.spatial.transform import Rotation
from scipy.misc import derivative
from scipy.integrate import quad, fixed_quad, quadrature
from scipy.optimize import minimize
from numpy.polynomial import Polynomial

from ament_index_python.packages import get_package_share_directory
print(f"Current Working Directory : {os.getcwd()}", flush=True)

image_path = '487_1723092780-883211914.png'
# image_path = '302_1723092774-710305664.png'
# image_path = '142_1723092769-375700928.png'

def create_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Directory {directory_path} created.")
    else:
        print(f"Directory {directory_path} already exists.")

class RBSC:
    def __init__(self, config_file='config.json'):
        self.config = self.load_config(config_file)

        self.yx_coords = None
        self.xy_coords = None
        self.__div = None

        self.func_poly4d = None
        self.func_poly3d = None
        self.func_log = None
        self.popt_poly4d = None
        self.popt_poly3d = None
        self.popt_log = None

        self.joint_yx_pixel = None
        self.joints_invtrans_xy = None
        self.base_origin_camera_samples = []
        self.base_origin_camera_xyz_fixed = None
        self.base_origin_filter_ready = False
        self.joint_kalman_states = None
        self.joint_kalman_covariances = None
        self.joint_kalman_timestamp_sec = None
        pass

    def load_config(self, config_file):

        package_share_directory = get_package_share_directory('estimation_pkg')
        config_path = os.path.join(package_share_directory, config_file)
        print(f'[postprocess.py] config.json PATH: {config_path}', flush=True)

        print(f'Load {config_file}', flush=True)
        print(f'===== json list ======', flush=True)
        with open(config_path, 'r') as f:
            config = json.load(f)
            # print
            for key, value in config.items():
                print(f'{key} : {value}', flush=True)
        print(f'===== json list end ===', flush=True)

        return config

    def smoothing(self, binary, k_size=5):
        # 빈틈을 채우기 위해 닫힘 연산 적용 (모폴로지 연산)
        kernel = np.ones((k_size, k_size), np.uint8)
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 외곽선을 찾아 근사화하지 않고 그대로 사용하여 매끄러운 곡선 유지
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # 빈 이미지 생성
        smooth_image = np.zeros_like(closed)

        # 근사화되지 않은 외곽선을 따라 그리기 (매끄러운 곡선 유지)
        cv2.drawContours(smooth_image, contours, -1, 255, thickness=cv2.FILLED)

        # 경계선 스무딩을 위해 블러링 적용 후 다시 이진화
        blurred = cv2.GaussianBlur(smooth_image, (15, 15), 0)
        _, final_smooth = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

        return final_smooth

    def rotation_matrix(self, coords, theta=-90):
        rotation_matrix = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))],
                                    [np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta))]])
        return np.dot(coords, rotation_matrix.T)
    def poly4d(self, x, a, b, c, d):
        return a * x ** 4 + b * x ** 3 + c * x ** 2 + d * x
        # return a * (x+2) ** 4 + b * (x+2) ** 3 + c * (x+2) ** 2 + d * (x+2)

    def poly3d(self, x, a, b, c):
        return a * x ** 3 + b * x ** 2 + c * x

    @staticmethod
    def fit_poly4d_through_origin(x, y):
        """Fit a*x^4+b*x^3+c*x^2+d*x using one linear least-squares solve."""
        design_matrix = np.column_stack((x ** 4, x ** 3, x ** 2, x))
        coefficients, _, rank, _ = np.linalg.lstsq(
            design_matrix, y, rcond=None
        )
        if rank < 4 or not np.all(np.isfinite(coefficients)):
            raise ValueError('Origin-constrained 2D quartic fit is ill-conditioned.')
        return coefficients
    def log(self, x, a, b):
        return a * np.log(b * (x + 1))
    def poly4d_sigmoid(self, x, a, b, c, d, L, k, x0):
        exp_term = np.clip(-k * (x - x0), -700, 700)
        return (a * x ** 4 + b * x ** 3 + c * x ** 2 + d * x) + L / (1 + np.exp(exp_term))
        # return (a * x **2 + b * x) + L / (1 + np.exp(-k * (x - x0)))

    # 곡선 양끝에 n개의 점 추가
    # y값을 기존값 그대로 쓸수도 있고, curve함수에 의해 수학적으로 구해지는 y를 쓸 수도 있다.
    def extend_curve(self, x, y, popt, rate=0.8, start_method='linear', end_method='curve',replace='origin'):
        """
        :param x: x values
        :param y: y values
        :param popt: coefficients of function
        :param rate: extend rate of number of points
        :param replace: The y value cna be used as it is (origin),
                        or the y obtained mathematically by the curve function can be used.
        :return: extended x, y values
        """
        interval =  (x.max() - x.min()) / len(x)
        n = len(x) * rate    # 전체 데이터 개수의 rate * 100 %만큼 연장
        n = int(n)
        extend_threshold = interval * n
        x_new_start = np.arange(x.min() - extend_threshold, x.min(), interval)
        x_new_end = np.arange(x.max(), x.max() + extend_threshold, interval)
        x_extended = np.concatenate([x_new_start, x, x_new_end])

        popt = popt
        if replace == 'origin':
            # default
            y_new_start = self.poly4d(x_new_start, *popt)
            y_new_end = self.poly4d(x_new_end, *popt)

            if start_method == 'linear':
                tangent = derivative(self.func_poly4d, x.min(), dx=1e-3)
                y_new_start = tangent * x_new_start
            elif start_method == 'curve':
                y_new_start = self.poly4d(x_new_start, *popt)

            if end_method == 'linear':
                tangent = derivative(self.func_poly4d, x.max(), dx=1e-3)
                y_new_end = tangent * x_new_end
            elif end_method == 'curve':
                y_new_end = self.poly4d(x_new_end, *popt)

            y_extended = np.concatenate([y_new_start, y, y_new_end])

        elif replace == 'replace':
            y_extended = self.poly4d(x_extended, *popt)

        return x_extended, y_extended

    def get_curve_length(self, x_start, x_end, popt='poly4d', method='fixed_quad', n=50):
        params = None
        if popt == 'poly4d':
            params = self.popt_poly4d
        # 피팅된 곡선 함수 정의 (curve_length 함수 내부에서)
        def fitted_curve(x):
            # return params[0] * x ** 4 + params[1] * x ** 3 + params[2] * x ** 2 + params[3] * x
            return self.poly4d(x, *params)

        # 곡선의 도함수를 수치적으로 계산하여 길이를 구함
        integrand = lambda x: np.sqrt(1 + derivative(fitted_curve, x, dx=1e-3) ** 2)
        # integrand = lambda x: np.sqrt(1 + self.dfdx_poly4d(x) ** 2)

        # length, _ = quad(integrand, x_start, x_end)
        if method == 'quad':
            length, _ = quad(integrand, x_start, x_end)
        elif method == 'quadrature':
            length, _ = quadrature(integrand, x_start, x_end)
        elif method == 'fixed_quad':
            length, _ = fixed_quad(integrand, x_start, x_end, n=n)

        return length

    # 주어진 곡선 길이에 대해 끝점(x1)을 찾는 함수
    def find_x_for_given_length(self, x_start, target_length):
        # 곡선 길이를 계산한 후, 주어진 길이와의 차이를 반환하는 함수 정의
        def length_difference(x_end):
            return abs(self.get_curve_length(x_start, x_end) - target_length)

        # 수치적으로 근을 찾기 위해 brentq 사용 (root-finding)
        # x_end = brentq(length_difference, x_start, x_start + target_length)  # 범위를 적절히 조절
        # return x_end

        result = minimize_scalar(
            length_difference,
            bounds=(x_start, x_start + target_length),
            method='bounded',
            options={'xatol': 1e-3}
        )
        return result.x

        # result = root_scalar(length_difference, bracket=[x_start, x_start + 10], method='brentq')
        # return result.root

    def find_closest_point(self, px, py):
        # 임의의 점 (px, py)에서 곡선까지의 거리 최소화
        def distance_from_curve(x):
            return (self.func_poly4d(x) - py) ** 2 + (x - px) ** 2

        result = minimize(distance_from_curve, px)
        return result.x[0]

    def range_normalization(self, coordinates):
        # ========== Normalization of width and height based on image size ==========
        # It should be reduced based on the length of the width or height, which is longer
        x_max = np.max(coordinates[:, 0])
        y_max = np.max(coordinates[:, 1])
        self.__div = max(x_max, y_max)
        # print(f'self.__div = {self.__div}')
        self.norm_xy_coords = np.copy(coordinates)
        self.norm_xy_coords[:, 0] = self.new_xy_coords[:, 0] / self.__div
        self.norm_xy_coords[:, 1] = self.new_xy_coords[:, 1] / self.__div

        return self.norm_xy_coords

    def range_denormalization(self, coordinates):

        return self.__div, self.norm_xy_coords

    def pixel_to_orthogonal_coordinate(self, binary):
        # ========== Transform image pixel coordinate to mathematical X-Y coordinate ==========
        self.yx_coords = np.column_stack(np.where(binary > 0)).astype(np.float64)
        # self.yx_coords = self.yx_coords[:-10]
        self.num_of_pixel_points = len(self.yx_coords)
        self.xy_coords = self.yx_coords[:, [1, 0]]  # x, y 열 위치 조정
        self.xy_coords[:, 1] = binary.shape[0] - self.xy_coords[:, 1]  # height - y

        # Set the point with the smallest y to the origin
        y_min_arg = np.argmin(self.xy_coords[:, 1])
        self.origin_xy = self.xy_coords[y_min_arg, :]
        self.new_xy_coords = self.xy_coords - self.origin_xy

        self.range_normalization(self.new_xy_coords)

        return self.norm_xy_coords

    def orthogonal_to_pixel_coordinate(self, points):
        # 3. Denormalization
        self.invtrans_xy_coords = points * self.__div

        # 4. Remove the offset that was moved to the origin
        self.extended_xy_coords = self.invtrans_xy_coords + self.origin_xy

        # 5. put back mathematical X-Y coordinate to image coordinate
        x = np.around(self.extended_xy_coords[:, 0])
        y = np.around(self.extended_xy_coords[:, 1])
        new_x, new_y = x.astype(int), y.astype(int)
        new_y = self.image_h - new_y
        # exceptions for values above image size
        valid_indices = (new_x >= 0) & (new_x < self.image_w) & (new_y >= 0) & (new_y < self.image_h)
        new_x = new_x[valid_indices]
        new_y = new_y[valid_indices]
        self.extended_curve = np.zeros((self.image_h, self.image_w))
        # get new extended skeleton curve
        self.extended_curve[new_y, new_x] = 1
        self.extended_skeleton_origin_yx = np.argwhere(self.extended_curve > 0)

        # Preserve the parametric curve order. np.where/np.argwhere only return
        # image scan order, which cannot be used as base-to-tip curve order.
        yx_pixel = np.column_stack((new_y, new_x))
        if len(yx_pixel) > 1:
            keep = np.concatenate((
                [True],
                np.any(np.diff(yx_pixel, axis=0) != 0, axis=1),
            ))
            yx_pixel = yx_pixel[keep]

        return yx_pixel

    def deproject_skeleton_to_3d(
            self,
            ordered_yx,
            depth_image,
            camera_intrinsics,
            depth_scale,
            roi_offset=(0, 0)):
        """Convert ordered ROI skeleton pixels to camera-frame 3D points."""
        if depth_image is None or camera_intrinsics is None:
            raise ValueError('Aligned depth image and camera intrinsics are required.')
        if depth_image.shape[:2] != self.extended_skeleton.shape:
            raise ValueError('Color ROI and aligned-depth ROI shapes must match.')
        if len(ordered_yx) == 0:
            raise ValueError('The extended skeleton contains no pixels.')

        v = ordered_yx[:, 0].astype(np.int64)
        u = ordered_yx[:, 1].astype(np.int64)
        raw_depth = depth_image[v, u].astype(np.float64)

        filter_config = self.config.get('filters', {}).get(
            'depth_neighborhood_median', {}
        )
        median_enabled = bool(filter_config.get('enabled', False))
        filtered_depth = raw_depth.copy()
        if median_enabled:
            kernel_size = int(filter_config.get('kernel_size', 5))
            min_valid_samples = int(
                filter_config.get('min_valid_samples', 3)
            )
            if kernel_size < 1 or kernel_size % 2 == 0:
                raise ValueError(
                    'Depth median kernel_size must be a positive odd number.'
                )
            if min_valid_samples < 1 or min_valid_samples > kernel_size ** 2:
                raise ValueError(
                    'Depth median min_valid_samples must be between 1 and '
                    'kernel_size squared.'
                )

            radius = kernel_size // 2
            padded_depth = np.pad(
                depth_image,
                radius,
                mode='constant',
                constant_values=0,
            )
            neighborhoods = np.lib.stride_tricks.sliding_window_view(
                padded_depth, (kernel_size, kernel_size)
            )[v, u].astype(np.float64, copy=False)
            neighborhood_valid = (
                np.isfinite(neighborhoods) & (neighborhoods > 0.0)
            )
            valid_counts = np.count_nonzero(
                neighborhood_valid, axis=(1, 2)
            )
            use_median = valid_counts >= min_valid_samples
            if np.any(use_median):
                valid_neighborhoods = np.where(
                    neighborhood_valid[use_median],
                    neighborhoods[use_median],
                    np.nan,
                )
                filtered_depth[use_median] = np.nanmedian(
                    valid_neighborhoods, axis=(1, 2)
                )

        self.skeleton_depth_raw_m = raw_depth * depth_scale
        self.skeleton_depth_filtered_m = filtered_depth * depth_scale
        depth_m = self.skeleton_depth_filtered_m

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        self.skeleton_depth_valid_mask = valid.copy()
        if np.count_nonzero(valid) < 5:
            raise ValueError('Too few valid depth samples on the extended skeleton.')

        v = v[valid]
        u = u[valid]
        depth_m = depth_m[valid]
        ordered_yx = ordered_yx[valid]

        roi_x, roi_y = roi_offset
        u_full = u.astype(np.float64) + roi_x
        v_full = v.astype(np.float64) + roi_y
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        x = (u_full - cx) * depth_m / fx
        y = (v_full - cy) * depth_m / fy
        points_xyz = np.column_stack((x, y, depth_m))

        # The 2D fitted curve may be generated in either direction. Preserve a
        # stable base-to-tip direction using the legacy base-origin estimate.
        origin_yx = np.array([
            self.image_h - self.origin_xy[1],
            self.origin_xy[0],
        ])
        if (np.linalg.norm(ordered_yx[0] - origin_yx) >
                np.linalg.norm(ordered_yx[-1] - origin_yx)):
            ordered_yx = ordered_yx[::-1]
            points_xyz = points_xyz[::-1]

        return ordered_yx, points_xyz

    def filter_base_origin_camera(self, base_origin_camera_xyz):
        """Optionally fix the camera-frame Base origin after an initial mean."""
        candidate = np.asarray(base_origin_camera_xyz, dtype=float)
        self.base_origin_camera_xyz_raw = candidate.copy()
        filter_config = self.config.get('filters', {}).get(
            'base_origin_initial_average', {}
        )
        enabled = bool(filter_config.get('enabled', False))
        if not enabled:
            self.base_origin_filter_ready = False
            self.base_origin_samples_collected = 0
            return candidate.copy()

        sample_count = int(filter_config.get('sample_count', 15))
        if sample_count < 1:
            raise ValueError(
                'Base-origin initial-average sample_count must be positive.'
            )

        if not hasattr(self, 'base_origin_camera_samples'):
            self.base_origin_camera_samples = []
        if not hasattr(self, 'base_origin_camera_xyz_fixed'):
            self.base_origin_camera_xyz_fixed = None
        if not hasattr(self, 'base_origin_filter_ready'):
            self.base_origin_filter_ready = False

        if self.base_origin_camera_xyz_fixed is None:
            self.base_origin_camera_samples.append(candidate.copy())
            samples = np.asarray(self.base_origin_camera_samples, dtype=float)
            origin = np.mean(samples, axis=0)
            if len(samples) >= sample_count:
                self.base_origin_camera_xyz_fixed = origin.copy()
                self.base_origin_filter_ready = True
                print(
                    'Base-origin initial average fixed after '
                    f'{len(samples)} samples.',
                    flush=True,
                )
        else:
            origin = self.base_origin_camera_xyz_fixed.copy()
            self.base_origin_filter_ready = True

        self.base_origin_samples_collected = len(
            self.base_origin_camera_samples
        )
        return origin

    def transform_camera_points_to_base(
            self, points_xyz_camera, transform_camera_from_base=None):
        """Express camera points relative to the estimated HRM base frame.

        ``transform_camera_from_base`` describes the base pose in the camera
        frame. It may be a 3x3 rotation matrix or a 4x4 homogeneous transform;
        only its rotation is used because the base translation is estimated
        from the first ordered skeleton point.
        """
        if len(points_xyz_camera) == 0:
            raise ValueError('No camera-frame 3D points are available.')

        if transform_camera_from_base is None:
            angle_y = float(self.config['base_in_camera_rotation_y_deg'])
            angle_z = float(self.config['base_in_camera_rotation_z_deg'])
            rotation_camera_from_base = (
                Rotation.from_euler('y', angle_y, degrees=True)
                * Rotation.from_euler('z', angle_z, degrees=True)
            )
        else:
            transform = np.asarray(
                transform_camera_from_base, dtype=float
            )
            if transform.shape == (4, 4):
                rotation_matrix = transform[:3, :3]
            elif transform.shape == (3, 3):
                rotation_matrix = transform
            else:
                raise ValueError(
                    'Camera-from-base transform must be 3x3 or 4x4.'
                )
            if not np.all(np.isfinite(rotation_matrix)):
                raise ValueError(
                    'Camera-from-base rotation contains non-finite values.'
                )
            rotation_camera_from_base = Rotation.from_matrix(rotation_matrix)

        # Rotation notation:
        #   ^C R_B : Base-frame coordinates -> Camera-frame coordinates
        #            Base orientation expressed in the Camera frame
        #
        #   ^B R_C : Camera-frame coordinates -> Base-frame coordinates
        #            Camera orientation expressed in the Base frame
        #   ^B p = ^B R _C * ^C p = (^C R _B)^T  * ^C p
        # For rotation matrices:
        #   ^B R_C = (^C R_B)^-1 = (^C R_B)^T
        rotation_base_from_camera = rotation_camera_from_base.inv()
        self.rotation_camera_from_base = (
            rotation_camera_from_base.as_matrix()
        )
        self.quaternion_camera_from_base_xyzw = (
            rotation_camera_from_base.as_quat()
        )
        self.rotation_base_from_camera = (
            rotation_base_from_camera.as_matrix()
        )
        self.base_origin_camera_xyz = self.filter_base_origin_camera(
            points_xyz_camera[0]
        )
        points_relative_camera = (
            points_xyz_camera - self.base_origin_camera_xyz
        )
        points_xyz_base = rotation_base_from_camera.apply(
            points_relative_camera
        )
        self.base_frame_id = self.config.get('base_frame_id', 'hrm_base')
        return points_xyz_base

    @staticmethod
    def fit_quartic_through_origin(s, values):
        """Fit a4*s^4 + a3*s^3 + a2*s^2 + a1*s with r(0)=0."""
        design_matrix = np.column_stack((s**4, s**3, s**2, s))
        coefficients, _, rank, _ = np.linalg.lstsq(
            design_matrix, values, rcond=None
        )
        if rank < 4 or not np.all(np.isfinite(coefficients)):
            raise ValueError('Origin-constrained quartic fit is ill-conditioned.')
        return np.concatenate((coefficients, [0.0]))

    @staticmethod
    def wrap_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def update_joint_kalman(
            self, index, measurement, dt, filter_config):
        """Update one constant-velocity angular state [q, q_dot]."""
        measurement_std = np.deg2rad(
            float(filter_config.get('measurement_std_deg', 1.0))
        )
        acceleration_std = np.deg2rad(float(filter_config.get(
            'process_acceleration_std_deg_s2', 30.0
        )))
        initial_angle_std = np.deg2rad(
            float(filter_config.get('initial_angle_std_deg', 5.0))
        )
        initial_velocity_std = np.deg2rad(
            float(filter_config.get('initial_velocity_std_deg_s', 20.0))
        )
        if min(
                measurement_std,
                acceleration_std,
                initial_angle_std,
                initial_velocity_std) <= 0.0:
            raise ValueError(
                'All joint Kalman standard deviations must be positive.'
            )

        state = self.joint_kalman_states[index]
        if not np.all(np.isfinite(state)):
            state[:] = (measurement, 0.0)
            self.joint_kalman_covariances[index] = np.diag((
                initial_angle_std ** 2,
                initial_velocity_std ** 2,
            ))
            return state.copy()

        transition = np.array([[1.0, dt], [0.0, 1.0]])
        process_covariance = acceleration_std ** 2 * np.array((
            (0.25 * dt ** 4, 0.5 * dt ** 3),
            (0.5 * dt ** 3, dt ** 2),
        ))
        predicted_state = transition @ state
        predicted_state[0] = self.wrap_angle(predicted_state[0])
        predicted_covariance = (
            transition @ self.joint_kalman_covariances[index]
            @ transition.T + process_covariance
        )

        innovation = self.wrap_angle(measurement - predicted_state[0])
        innovation_variance = (
            predicted_covariance[0, 0] + measurement_std ** 2
        )
        kalman_gain = predicted_covariance[:, 0] / innovation_variance
        updated_state = predicted_state + kalman_gain * innovation
        updated_state[0] = self.wrap_angle(updated_state[0])
        observation = np.array([[1.0, 0.0]])
        updated_covariance = (
            np.eye(2) - kalman_gain[:, None] @ observation
        ) @ predicted_covariance

        self.joint_kalman_states[index] = updated_state
        self.joint_kalman_covariances[index] = 0.5 * (
            updated_covariance + updated_covariance.T
        )
        return updated_state.copy()

    def project_segment_directions_to_joint_planes(self, timestamp_sec=None):
        """Project raw segment directions onto sequential DH joint planes.

        The first of the 19 geometric segments is the fixed proximal ``os``
        segment. The remaining 18 directions correspond to q1 through q18.
        Table 1 is interpreted with the orientation recursion
        R_B_i = R_B_(i-1) * Rx(alpha_(i-1)) * Rz(q_i). The first
        joint is pan (alpha_0=0), followed by alternating +90/-90 degree
        twists. All saved axes and directions are expressed in ``hrm_base``.
        """
        all_raw_directions = self.segment_directions_xyz
        num_segments = len(all_raw_directions)
        num_joints = int(self.config['num_of_bending_joints'])
        if num_segments != num_joints + 1:
            raise ValueError(
                'The geometric segment count must equal the bending-joint '
                'count plus the fixed proximal segment.'
            )

        self.fixed_base_segment_direction_raw_xyz = (
            all_raw_directions[0].copy()
        )
        raw_directions = all_raw_directions[1:]
        self.joint_segment_directions_raw_xyz = raw_directions.copy()

        twist_degrees = np.zeros(num_joints, dtype=float)
        if num_joints > 1:
            indices = np.arange(1, num_joints)
            twist_degrees[1:] = np.where(indices % 2 == 1, 90.0, -90.0)

        projected_directions = np.empty_like(raw_directions)
        joint_axes = np.empty_like(raw_directions)
        residuals = np.empty(num_joints, dtype=float)
        joint_angles = np.empty(num_joints, dtype=float)
        joint_frame_rotations = np.empty((num_joints, 3, 3), dtype=float)

        rotation_base_from_previous = Rotation.identity()
        base_x = np.array([1.0, 0.0, 0.0])
        base_z = np.array([0.0, 0.0, 1.0])

        for index, (direction_raw, twist_degree) in enumerate(zip(
                raw_directions, twist_degrees)):
            rotation_base_from_prejoint = (
                rotation_base_from_previous
                * Rotation.from_euler('x', twist_degree, degrees=True)
            )
            joint_axis = rotation_base_from_prejoint.apply(base_z)
            zero_angle_direction = rotation_base_from_prejoint.apply(base_x)

            residual = float(np.dot(direction_raw, joint_axis))
            direction_projected = direction_raw - residual * joint_axis
            projected_norm = np.linalg.norm(direction_projected)
            if projected_norm <= 1e-9:
                raise ValueError(
                    f'Segment {index + 2} direction is parallel to its '
                    'estimated DH joint axis.'
                )
            direction_projected /= projected_norm

            sine = np.dot(
                joint_axis,
                np.cross(zero_angle_direction, direction_projected),
            )
            cosine = np.dot(zero_angle_direction, direction_projected)
            joint_angle = np.arctan2(sine, cosine)
            rotation_base_from_current = (
                rotation_base_from_prejoint
                * Rotation.from_euler('z', joint_angle)
            )

            projected_directions[index] = direction_projected
            joint_axes[index] = joint_axis
            residuals[index] = residual
            joint_angles[index] = joint_angle
            joint_frame_rotations[index] = (
                rotation_base_from_current.as_matrix()
            )
            rotation_base_from_previous = rotation_base_from_current

        # Preserve all 19 visualization directions. Segment zero is the fixed
        # proximal link and therefore has the Base-frame +X direction; the
        # remaining 18 entries are the model-constrained moving segments.
        self.joint_segment_directions_projected_xyz = projected_directions
        self.segment_directions_projected_xyz = np.vstack((
            base_x,
            projected_directions,
        ))
        self.segment_projection_residuals = residuals
        self.segment_projection_residual_vectors_xyz = (
            residuals[:, None] * joint_axes
        )
        self.dh_joint_axes_xyz = joint_axes
        self.dh_twist_degrees = twist_degrees
        self.dh_joint_angles_projected_rad = joint_angles
        self.dh_joint_angles_projected_degree = np.degrees(joint_angles)
        self.dh_joint_frame_rotations = joint_frame_rotations
        self.dh_joint_types = [
            'pan' if index % 2 == 0 else 'tilt'
            for index in range(num_joints)
        ]

        kalman_config = self.config.get('filters', {}).get(
            'joint_kalman', {}
        )
        timestamp_is_valid = (
            timestamp_sec is not None and np.isfinite(timestamp_sec)
        )
        kalman_enabled = bool(
            kalman_config.get('enabled', False) and timestamp_is_valid
        )
        self.joint_kalman_filter_enabled = kalman_enabled
        filtered_joint_angles = joint_angles.copy()
        filtered_joint_velocities = np.zeros(num_joints, dtype=float)

        if kalman_enabled:
            state_shape_changed = (
                self.joint_kalman_states is None
                or self.joint_kalman_states.shape != (num_joints, 2)
            )
            if state_shape_changed:
                self.joint_kalman_states = np.full(
                    (num_joints, 2), np.nan, dtype=float
                )
                self.joint_kalman_covariances = np.zeros(
                    (num_joints, 2, 2), dtype=float
                )
                self.joint_kalman_timestamp_sec = None

            if self.joint_kalman_timestamp_sec is None:
                dt = 0.0
            else:
                dt = float(timestamp_sec - self.joint_kalman_timestamp_sec)
                if not np.isfinite(dt) or dt <= 0.0:
                    self.joint_kalman_states.fill(np.nan)
                    self.joint_kalman_covariances.fill(0.0)
                    dt = 0.0

            for index, measurement in enumerate(joint_angles):
                state = self.update_joint_kalman(
                    index,
                    measurement,
                    dt,
                    kalman_config,
                )
                filtered_joint_angles[index] = state[0]
                filtered_joint_velocities[index] = state[1]
            self.joint_kalman_timestamp_sec = float(timestamp_sec)
        else:
            self.joint_kalman_states = None
            self.joint_kalman_covariances = None
            self.joint_kalman_timestamp_sec = None

        # Reconstruct DH-constrained directions once more using filtered q.
        # Raw and projection-only vectors above remain unchanged for comparison.
        filtered_joint_directions = np.empty_like(raw_directions)
        filtered_frame_rotations = np.empty(
            (num_joints, 3, 3), dtype=float
        )
        rotation_base_from_previous = Rotation.identity()
        for index, (joint_angle, twist_degree) in enumerate(zip(
                filtered_joint_angles, twist_degrees)):
            rotation_base_from_prejoint = (
                rotation_base_from_previous
                * Rotation.from_euler('x', twist_degree, degrees=True)
            )
            rotation_base_from_current = (
                rotation_base_from_prejoint
                * Rotation.from_euler('z', joint_angle)
            )
            filtered_joint_directions[index] = (
                rotation_base_from_current.apply(base_x)
            )
            filtered_frame_rotations[index] = (
                rotation_base_from_current.as_matrix()
            )
            rotation_base_from_previous = rotation_base_from_current

        self.dh_joint_angles_filtered_rad = filtered_joint_angles
        self.dh_joint_angles_filtered_degree = np.degrees(
            filtered_joint_angles
        )
        self.dh_joint_angular_velocities_filtered_rad_s = (
            filtered_joint_velocities
        )
        self.dh_joint_frame_rotations_filtered = filtered_frame_rotations
        self.joint_segment_directions_filtered_xyz = (
            filtered_joint_directions
        )
        self.segment_directions_filtered_xyz = np.vstack((
            base_x,
            filtered_joint_directions,
        ))
        self.segment_center_frame_rotations_filtered = np.concatenate((
            np.eye(3, dtype=float)[None, :, :],
            filtered_frame_rotations,
        ), axis=0)

        # Relative arrays retain one entry per physical joint. The inactive
        # axis of each alternating one-DOF joint is explicitly zero.
        self.pan_relative_rad = np.zeros(num_joints, dtype=float)
        self.tilt_relative_rad = np.zeros(num_joints, dtype=float)
        self.pan_relative_rad[0::2] = filtered_joint_angles[0::2]
        self.tilt_relative_rad[1::2] = filtered_joint_angles[1::2]
        self.pan_relative_velocity_rad_s = np.zeros(num_joints, dtype=float)
        self.tilt_relative_velocity_rad_s = np.zeros(num_joints, dtype=float)
        self.pan_relative_velocity_rad_s[0::2] = (
            filtered_joint_velocities[0::2]
        )
        self.tilt_relative_velocity_rad_s[1::2] = (
            filtered_joint_velocities[1::2]
        )

        # Absolute pan/tilt describe each filtered outgoing segment direction
        # in the fixed HRM Base axes: X axial, Y pan, and Z tilt.
        self.pan_absolute_rad = np.arctan2(
            filtered_joint_directions[:, 1],
            filtered_joint_directions[:, 0],
        )
        self.tilt_absolute_rad = np.arctan2(
            filtered_joint_directions[:, 2],
            np.hypot(
                filtered_joint_directions[:, 0],
                filtered_joint_directions[:, 1],
            ),
        )

    def reconstruct_3d_segments(self, points_xyz, timestamp_sec=None):
        """Fit a normalized parametric 3D quartic and reconstruct segments."""
        finite = np.all(np.isfinite(points_xyz), axis=1)
        points = points_xyz[finite]
        if len(points) < 5:
            raise ValueError('At least five valid 3D points are required.')

        if len(points) > 1:
            keep = np.concatenate((
                [True],
                np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9,
            ))
            points = points[keep]

        num_segments = int(self.config['num_of_segments'])
        num_bending_joints = int(self.config['num_of_bending_joints'])
        num_joint_pairs = int(self.config['num_of_joint_pairs'])
        num_pan_joints = int(self.config['num_of_pan_joints'])
        num_tilt_joints = int(self.config['num_of_tilt_joints'])
        if num_segments != num_bending_joints + 1:
            raise ValueError(
                'num_of_segments must equal num_of_bending_joints + 1.'
            )
        if num_bending_joints != 2 * num_joint_pairs:
            raise ValueError(
                'num_of_bending_joints must equal 2 * num_of_joint_pairs.'
            )
        if (num_pan_joints != num_joint_pairs or
                num_tilt_joints != num_joint_pairs):
            raise ValueError(
                'The pan and tilt joint counts must equal '
                'num_of_joint_pairs.'
            )
        segment_length_m = float(self.config['length_of_segment']) * 1e-3
        proximal_length_m = (
            float(self.config['proximal_offset_length']) * 1e-3
        )
        joint_spacing_m = (
            float(self.config['bending_joint_spacing']) * 1e-3
        )
        distal_length_m = (
            float(self.config['distal_offset_length']) * 1e-3
        )
        self.segment_lengths_m = np.concatenate((
            [proximal_length_m],
            np.full(num_bending_joints - 1, joint_spacing_m),
            [distal_length_m],
        ))
        if len(self.segment_lengths_m) != num_segments:
            raise ValueError('The configured DH link lengths are inconsistent.')
        if (np.any(self.segment_lengths_m <= 0.0) or
                segment_length_m <= 0.0):
            raise ValueError('All configured segment lengths must be positive.')
        if not np.allclose(
                self.segment_lengths_m,
                segment_length_m,
                rtol=0.0,
                atol=1e-12):
            raise ValueError(
                'This reconstruction currently requires os, l, and le to '
                'equal length_of_segment.'
            )
        self.hardware_length_m = float(np.sum(self.segment_lengths_m))
        if self.hardware_length_m <= 0.0:
            raise ValueError('The configured hardware length must be positive.')

        # The Base transform already supplies the common origin. Anchor the
        # constrained curve exactly at it instead of following first-point
        # depth jitter on every frame.
        points = points.copy()
        points[0] = 0.0
        self.curve_base_xyz = np.zeros(3, dtype=float)
        points_normalized = points / self.hardware_length_m
        raw_ds_normalized = np.linalg.norm(
            np.diff(points_normalized, axis=0), axis=1
        )
        raw_length_normalized = np.sum(raw_ds_normalized)
        if (not np.isfinite(raw_length_normalized) or
                raw_length_normalized <= 1e-9):
            raise ValueError('The ordered 3D skeleton has near-zero length.')

        raw_cumulative_length = np.concatenate(
            ([0.0], np.cumsum(raw_ds_normalized))
        )
        s_raw = raw_cumulative_length / raw_length_normalized

        self.points_xyz_normalized = points_normalized
        self.coef_x = self.fit_quartic_through_origin(
            s_raw, points_normalized[:, 0]
        )
        self.coef_y = self.fit_quartic_through_origin(
            s_raw, points_normalized[:, 1]
        )
        self.coef_z = self.fit_quartic_through_origin(
            s_raw, points_normalized[:, 2]
        )

        dense_count = int(self.config.get('curve_dense_samples', 1001))
        if dense_count < 2:
            raise ValueError('curve_dense_samples must be at least 2.')
        self.s_dense = np.linspace(0.0, 1.0, dense_count)
        self.curve_dense_normalized = np.column_stack((
            np.polyval(self.coef_x, self.s_dense),
            np.polyval(self.coef_y, self.s_dense),
            np.polyval(self.coef_z, self.s_dense),
        ))
        self.curve_dense_xyz = (
            self.curve_dense_normalized * self.hardware_length_m
            + self.curve_base_xyz
        )

        dense_ds = np.linalg.norm(
            np.diff(self.curve_dense_normalized, axis=0), axis=1
        )
        dense_cumulative_length = np.concatenate(([0.0], np.cumsum(dense_ds)))
        curve_length_normalized = dense_cumulative_length[-1]
        if (not np.isfinite(curve_length_normalized) or
                curve_length_normalized <= 1e-9):
            raise ValueError('The fitted 3D curve has near-zero length.')

        # Depth and endpoint extraction can make the measured fitted curve a
        # few percent too long or too short. The physical centerline length is
        # fixed, so apply one isotropic Base-anchored scale before selecting
        # segment boundaries. Directions and joint angles are unchanged, while
        # P0...P19 and the model FK now share the same 82.27 mm arc length.
        self.fitted_curve_length_before_hardware_scaling_3d = (
            curve_length_normalized * self.hardware_length_m
        )
        self.hardware_length_scale = 1.0 / curve_length_normalized
        self.coef_x *= self.hardware_length_scale
        self.coef_y *= self.hardware_length_scale
        self.coef_z *= self.hardware_length_scale
        self.curve_dense_normalized *= self.hardware_length_scale
        dense_cumulative_length *= self.hardware_length_scale
        curve_length_normalized = dense_cumulative_length[-1]
        self.curve_dense_xyz = (
            self.curve_dense_normalized * self.hardware_length_m
            + self.curve_base_xyz
        )

        # Remove zero-length interpolation intervals while preserving endpoints.
        interp_keep = np.concatenate((
            [True],
            np.diff(dense_cumulative_length) > 1e-12,
        ))
        interp_length = dense_cumulative_length[interp_keep]
        interp_s = self.s_dense[interp_keep]

        hardware_boundary_lengths_m = np.concatenate((
            [0.0],
            np.cumsum(self.segment_lengths_m),
        ))
        target_lengths = (
            hardware_boundary_lengths_m / self.hardware_length_m
        )
        self.s_segment = np.interp(target_lengths, interp_length, interp_s)
        self.segment_points_xyz_normalized = np.column_stack((
            np.polyval(self.coef_x, self.s_segment),
            np.polyval(self.coef_y, self.s_segment),
            np.polyval(self.coef_z, self.s_segment),
        ))
        self.segment_points_xyz = (
            self.segment_points_xyz_normalized * self.hardware_length_m
            + self.curve_base_xyz
        )

        # Sample once more at the arc-length center of each of the 19
        # reconstructed segments. These are distinct from the 20 boundaries.
        hardware_center_lengths_m = (
            hardware_boundary_lengths_m[:-1]
            + 0.5 * self.segment_lengths_m
        )
        center_target_lengths = (
            hardware_center_lengths_m / self.hardware_length_m
        )
        self.s_segment_center = np.interp(
            center_target_lengths, interp_length, interp_s
        )
        self.segment_center_points_xyz_normalized = np.column_stack((
            np.polyval(self.coef_x, self.s_segment_center),
            np.polyval(self.coef_y, self.s_segment_center),
            np.polyval(self.coef_z, self.s_segment_center),
        ))
        self.segment_center_points_xyz = (
            self.segment_center_points_xyz_normalized * self.hardware_length_m
            + self.curve_base_xyz
        )

        # Analytic tangent of the fitted parametric curve at all 20 segment
        # boundary points. Isotropic normalization does not change direction.
        tangent_vectors = np.column_stack((
            np.polyval(np.polyder(self.coef_x), self.s_segment),
            np.polyval(np.polyder(self.coef_y), self.s_segment),
            np.polyval(np.polyder(self.coef_z), self.s_segment),
        ))
        tangent_norms = np.linalg.norm(tangent_vectors, axis=1)
        if np.any(tangent_norms <= 1e-12):
            raise ValueError('The fitted 3D curve has a zero tangent.')
        self.segment_tangents_xyz = tangent_vectors / tangent_norms[:, None]

        center_tangent_vectors = np.column_stack((
            np.polyval(np.polyder(self.coef_x), self.s_segment_center),
            np.polyval(np.polyder(self.coef_y), self.s_segment_center),
            np.polyval(np.polyder(self.coef_z), self.s_segment_center),
        ))
        center_tangent_norms = np.linalg.norm(center_tangent_vectors, axis=1)
        if np.any(center_tangent_norms <= 1e-12):
            raise ValueError(
                'The fitted 3D curve has a zero tangent at a segment center.'
            )
        self.segment_center_tangents_xyz = (
            center_tangent_vectors / center_tangent_norms[:, None]
        )

        self.segment_vectors_xyz = np.diff(self.segment_points_xyz, axis=0)
        self.segment_lengths_3d = np.linalg.norm(self.segment_vectors_xyz, axis=1)
        if np.any(self.segment_lengths_3d <= 1e-12):
            raise ValueError('A reconstructed segment has near-zero length.')
        self.segment_directions_xyz = (
            self.segment_vectors_xyz / self.segment_lengths_3d[:, None]
        )
        self.project_segment_directions_to_joint_planes(timestamp_sec)

        self.raw_curve_length_3d = (
            raw_length_normalized * self.hardware_length_m
        )
        self.curve_length_3d = (
            curve_length_normalized * self.hardware_length_m
        )
        self.raw_s_3d = s_raw
        return self.segment_points_xyz, self.segment_directions_xyz

    def get_joints(self):
        # ============= find junction points ============
        # 끝점의 기울기 - test
        # self.tip_tangent = derivative(self.func_poly4d, self.x1, dx=1e-3)
        # self.tip_angle_rad = np.arctan(self.tip_tangent)
        # self.tip_angle_deg = np.degrees(self.tip_angle_rad)
        # print(f'tip_angle (deg) = {self.tip_angle_deg}')

        ###########################################################################
        # get each angle of joints
        num_of_segments = self.config.get('num_of_segments')
        length_of_segment = self.config.get('length_of_segment')  # mm
        self.joint_x = np.zeros(num_of_segments)
        # print('============ get joints_x ================')
        # print(f'total len = {self.curve_length}')

        target_len = self.curve_length / float(2 * num_of_segments)

        x1 = self.find_x_for_given_length(self.start_point[0], target_length=target_len)
        self.joint_x[0] = x1

        for i in range(1, num_of_segments):
            target_len = self.curve_length / float(num_of_segments)
            x1 = self.find_x_for_given_length(self.joint_x[i-1], target_length=target_len)
            self.joint_x[i] = x1
            # print(f'i={i} || target_len={target_len} | x0={self.joint_x[i-1]} -> x1={self.joint_x[i]}')

        ###########################################################################
        # self.joint_y = self.poly4d(self.joint_x, *self.popt_poly4d)
        self.joint_y = self.func_poly4d(self.joint_x)
        self.joints_xy = np.column_stack((self.joint_x, self.joint_y))

        self.joint_tangents = np.array([derivative(self.func_poly4d, x0, dx=1e-3) for x0 in self.joint_x])
        self.joint_angle = np.arctan(self.joint_tangents)
        self.joint_angle_degree = np.degrees(self.joint_angle)

        # first(0) joint is unstable compared to another joints
        # 첫번째 segment는 오차가 있어서 점사이의 벡터를 가지고 각도를 따로 정의함
        vector_seg1_to_seg2 = [self.joints_xy[1,0]-self.joints_xy[0,0], self.joints_xy[1,1]-self.joints_xy[0,1]]
        tangent = vector_seg1_to_seg2[1] / vector_seg1_to_seg2[0]
        theta = np.arctan(tangent)
        joint_0_theta = theta/2.0
        self.joint_angle[0] = joint_0_theta
        self.joint_angle_degree[0] = np.degrees(self.joint_angle[0])

        diff_joint_angle = np.diff(self.joint_angle)
        self.joint_angle_relative = np.insert(diff_joint_angle, 0, self.joint_angle[0])
        self.joint_angle_relative_degree = np.degrees(self.joint_angle_relative)

        # print(f'joints = {self.joints_xy}')
        # print(f'joint_tangent = {self.joint_tangents}')
        # print(f'joint_angle (rad) = {self.joint_angle}')
        # print(f'joint_angle (deg) = {self.joint_angle_degree}')

        '''
        @ TODO
        have to add the operation for estimating position of each joints
        using each estimated angles

        '''

    def postprocess(
            self,
            image,
            color_encoding='bgr8',
            depth_image=None,
            camera_intrinsics=None,
            depth_scale=1.0,
            roi_offset=(0, 0),
            transform_camera_from_base=None,
            timestamp_sec=None,
            binary_thresh=120,
            filfinder_flag=False):
        timing_start = time.perf_counter()
        try:
            # ========== post processing ==========
            # 1. read as grayscale
            self.image = image
            if image is None or image.size == 0:
                # print("Error: image : {image}")
                return

            # Keep the camera's native RGB/BGR order. Converting the complete
            # incoming frame to BGR in the ROS callback unnecessarily copies
            # hundreds of kilobytes before the ROI is selected.
            normalized_color_encoding = color_encoding.lower()
            if normalized_color_encoding == 'rgb8':
                hsv_code = cv2.COLOR_RGB2HSV
                gray_code = cv2.COLOR_RGB2GRAY
            elif normalized_color_encoding == 'bgr8':
                hsv_code = cv2.COLOR_BGR2HSV
                gray_code = cv2.COLOR_BGR2GRAY
            else:
                raise ValueError(
                    f'Unsupported color encoding: {color_encoding}. '
                    'Expected rgb8 or bgr8.'
                )

            # 2. 색상 제거 전처리: HSV 색공간 변환
            hsv = cv2.cvtColor(image, hsv_code)

            # 붉은색 범위 지정 (두 구간으로 나눔)
            lower_red1 = np.array([0, 50, 50])   # BGR에서 R이 강하고 B나 G가 상대적으로 작음
            upper_red1 = np.array([255, 220, 220])
            # lower_red1 = np.array([0, 50, 50])
            # upper_red1 = np.array([50, 200, 200])
            # lower_red2 = np.array([100, 10, 10])
            # upper_red2 = np.array([200, 255, 255])

            # 파란색 범위 지정
            lower_blue = np.array([100, 70, 50])
            upper_blue = np.array([130, 255, 255])

            # 마스크 생성
            # mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # 전체 마스크: 붉은색 또는 파란색
            mask = cv2.bitwise_or(mask_red, mask_blue)

            # Convert only once to grayscale and mask there. Creating a second
            # full BGR image and assigning three channels produced the same
            # binary result but caused avoidable memory traffic every frame.
            self.gray_image = cv2.cvtColor(image, gray_code)
            self.gray_image[mask != 0] = 0
            # self.gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # self.gray_image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
            self.image_w, self.image_h = self.gray_image.shape[1], self.gray_image.shape[0]

            # 2. Binarization
            # 임계값 설정 (예: 127)
            thresh_value = binary_thresh
            _, self.binary_image = cv2.threshold(self.gray_image, thresh_value, 255, cv2.THRESH_BINARY)
            timing_color_mask_done = time.perf_counter()

            # 3. Find connected commponets
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(self.binary_image, connectivity=8)
            if num_labels < 1:
                return

            # 4. Find the largest componnet
            max_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])

            # 5. Leave only the largest commpent (it is body)
            self.body_image_origin = (labels == max_label).astype(np.uint8) * 255
            self.body_image = self.smoothing(binary=self.body_image_origin, k_size=15)
            timing_body_mask_done = time.perf_counter()

            # print(f'{self.image} / {self.image.shape} / {self.image.size} ')
            # print(f'{self.body_image} / {self.body_image.shape} / {self.body_image.size} ')
            # ========== Find skeleton of backbone ==========
            # perform skeletonization
            self.skeleton = skeletonize(self.body_image, method='lee')
            timing_skeleton_done = time.perf_counter()

            ####################################################################
            '''
            TODO
            샘플링 타임에 가장 시간 많이쓰는 구간
            없애던지 대체해야될것같기도... smoothing때문에 안해도 될것 같기도...
            '''
            # Find the longest curve of backbone
            if filfinder_flag == True:
                fil = FilFinder2D(self.skeleton, distance=250 * u.pc, mask=self.skeleton)
                fil.preprocess_image(flatten_percent=85)
                fil.create_mask(border_masking=True, verbose=False, use_existing_mask=True)
                fil.medskel(verbose=False)
                fil.analyze_skeletons(branch_thresh=40 * u.pix, skel_thresh=30 * u.pix, prune_criteria='length')
                self.longest_backbone_image = fil.skeleton_longpath
            else:
                self.longest_backbone_image = np.copy(self.skeleton)
            ####################################################################

            self.pixel_to_orthogonal_coordinate(self.longest_backbone_image)
            timing_preprocess_done = time.perf_counter()
        except Exception as e:
            print(f'postprocess error : {e}', flush=True)
            return

        # ========== Curve fitting ==========
        try:
            # 데이터 가중치 설정
            # weights = np.ones_like(self.yx_coords[:, 0])
            # sigma = 0.1
            # num = 10
            # weights[0:num] = sigma
            # weights[-num:] = sigma
            # sigma = 1 / weights

            # 1. Rotate the points clockwise 90 degree
            self.trans_xy_coords = self.rotation_matrix(self.norm_xy_coords, theta=-90)

            # 2. The quartic is linear in its coefficients. A direct least-
            # squares solve is equivalent to curve_fit here and avoids its
            # iterative optimizer and covariance calculation.
            self.temp_popt_poly4d = self.fit_poly4d_through_origin(
                self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1]
            )
            # self.popt_poly4d, _ = curve_fit(self.poly4d, self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], sigma=sigma, maxfev=2000)
            self.fitted_y_poly4d = self.poly4d(self.trans_xy_coords[:, 0], *self.temp_popt_poly4d)
            self.func_poly4d = lambda x: self.poly4d(x, *self.temp_popt_poly4d)
            self.dfdx_poly4d = lambda x: self.temp_popt_poly4d[0]*4*x**3 + self.temp_popt_poly4d[1]*3*x**2 + self.temp_popt_poly4d[2]*2*x + self.temp_popt_poly4d[3]
            # self.popt_poly3d, _ = curve_fit(self.poly3d, self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], maxfev=2000)
            # self.fitted_y_poly3d = self.poly3d(self.trans_xy_coords[:, 0], *self.popt_poly3d)
            #
            # self.popt_log, _ = curve_fit(self.log, self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], maxfev=2000)
            # self.fitted_y_log = self.log(self.trans_xy_coords[:, 0], *self.popt_log)

            # self.popt_poly4d_sigmoid, _ = curve_fit(self.poly4d_sigmoid, self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], maxfev=2000)
            # self.fitted_y_poly4d_sigmoid = self.poly4d_sigmoid(self.trans_xy_coords[:, 0], *self.popt_poly4d_sigmoid)

            # ============================================================
            ''' 
            TODO
            Extrapolation for getting full center line
            1. find startpoint(endpoint) of skeleton
            2. set startpoint as origin (get offset of x, y)
            3. curvefit
            4. put back origin to startpoint in 2.
            5. extrapolation 
            6. get center line of object
            '''
            # 1. Extraploation
            ext_coords_x, ext_coords_y = self.extend_curve(self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], self.temp_popt_poly4d, rate=0.2)

            # Import
            # update curvefit coefficients
            """
            TODO
            곡선 연장후 재피팅 안해줄 경우 skeleton 끝이 테두리로 삐져나갔을 때 대처가 안됌
            """
            self.popt_poly4d = self.fit_poly4d_through_origin(
                ext_coords_x, ext_coords_y
            )
            self.fitted_y_poly4d = self.poly4d(ext_coords_x, *self.popt_poly4d)
            self.func_poly4d = lambda x: self.poly4d(x, *self.popt_poly4d)

            # Re-sample in curve-parameter order. The original skeleton pixels
            # arrive in image scan order, so they cannot be used directly as a
            # base-to-tip sequence for depth deprojection.
            ordered_ext_x = np.linspace(
                np.min(ext_coords_x),
                np.max(ext_coords_x),
                max(len(ext_coords_x), 2),
            )
            ordered_ext_y = self.func_poly4d(ordered_ext_x)
            self.extended_norm_coords = np.column_stack((
                ordered_ext_x,
                ordered_ext_y,
            ))

            # 2. Inverse transformation : rotate the points CCW 90 degree
            self.invtrans_norm_xy_coords = self.rotation_matrix(self.extended_norm_coords, theta=90)

            ordered_extended_yx = self.orthogonal_to_pixel_coordinate(
                self.invtrans_norm_xy_coords
            )

            # Remove curve points outside the boundary
            self.extended_skeleton = np.logical_and(
                self.body_image != 0, self.extended_curve != 0
            )
            inside_body = self.extended_skeleton[
                ordered_extended_yx[:, 0], ordered_extended_yx[:, 1]
            ] > 0
            ordered_extended_yx = ordered_extended_yx[inside_body]
            timing_curve_2d_done = time.perf_counter()

            self.extended_yx_coords, self.points_xyz_camera = (
                self.deproject_skeleton_to_3d(
                    ordered_extended_yx,
                    depth_image,
                    camera_intrinsics,
                    depth_scale,
                    roi_offset,
                )
            )
            timing_deprojection_done = time.perf_counter()
            self.points_xyz_base = self.transform_camera_points_to_base(
                self.points_xyz_camera,
                transform_camera_from_base,
            )
            # Downstream reconstruction is expressed entirely in the fixed
            # HRM base-frame axes. Keep this alias for existing consumers.
            self.points_xyz = self.points_xyz_base
            self.reconstruct_3d_segments(
                self.points_xyz, timestamp_sec=timestamp_sec
            )
            timing_reconstruction_done = time.perf_counter()
            self.last_stage_times_ms = {
                'preprocess': (
                    timing_preprocess_done - timing_start
                ) * 1e3,
                'color_mask': (
                    timing_color_mask_done - timing_start
                ) * 1e3,
                'body_mask': (
                    timing_body_mask_done - timing_color_mask_done
                ) * 1e3,
                'skeleton': (
                    timing_skeleton_done - timing_body_mask_done
                ) * 1e3,
                'coordinates': (
                    timing_preprocess_done - timing_skeleton_done
                ) * 1e3,
                'curve_2d': (
                    timing_curve_2d_done - timing_preprocess_done
                ) * 1e3,
                'deprojection': (
                    timing_deprojection_done - timing_curve_2d_done
                ) * 1e3,
                'reconstruction_3d': (
                    timing_reconstruction_done - timing_deprojection_done
                ) * 1e3,
                'total': (
                    timing_reconstruction_done - timing_start
                ) * 1e3,
            }
            return True

        except Exception as e:
            print(f"postprocess() error : {e}", flush=True)
            return None

    def draw_arrows(self, image):
        try:
            # Compute arrow vectors
            frame = image
            rads = self.joint_angle + np.pi / 2
            arrow_length = 15
            u = np.cos(rads) * arrow_length
            v = np.sin(rads) * arrow_length
            v = -v  # Reverse v to match the original behavior

            # Draw joint points and arrows
            pixel_yx = np.flip(self.joint_yx_pixel, axis=0)
            for point in pixel_yx:
                cv2.circle(frame, (int(point[1]), int(point[0])), 2, (0, 0, 255), -1)  # Red dots for joints

            for i in range(len(u)):
                start_point = (int(pixel_yx[i, 1]), int(pixel_yx[i, 0]))  # yx needs to be flipped for cv2
                end_point = (int(start_point[0] + u[i]), int(start_point[1] + v[i]))  # u and v provide direction
                # Draw the arrow line for each point
                cv2.arrowedLine(frame, start_point, end_point, (255, 0, 0), 2, tipLength=0.3)

            return frame
        except Exception as e:
            # print(f'draw_arrows() error: {e}', flush=True)
            return

    def plot_save(self, save_dir):
        # results_0.4mm
        # rads = -np.deg2rad(self.joint_angle_degree)
        rads = self.joint_angle + np.pi / 2
        # print(f'rads = {rads}')
        arrow_length = 15
        u = np.cos(rads) * arrow_length
        v = np.sin(rads) * arrow_length
        v = -v
        vector_scale = 1
        plt.figure(figsize=(10, 5))
        # plt.subplot(2, 3, 6)
        plt.title('Grayscale Image')
        plt.imshow(self.gray_image, cmap='gray')
        pixel_yx = np.flip(self.joint_yx_pixel, axis=0)
        plt.scatter(pixel_yx[:, 1], pixel_yx[:, 0], color='red', s=4, label='Joint Point')
        plt.quiver(pixel_yx[:, 1], pixel_yx[:, 0],
                   u, v,
                   angles='xy',
                   scale_units='xy',
                   scale=vector_scale,
                   width=0.005,
                   headwidth=2,
                   headlength=2,
                   headaxislength=2,
                   color='blue'
                   )
        plt.axis('off')

        # 이미지를 파일로 저장
        dir_name = save_dir
        image_filename = os.path.basename(self.image_path)
        plt.savefig(os.path.join(dir_name, image_filename), bbox_inches='tight', pad_inches=0)

    def show(self):
        # 결과 출력
        plt.figure(figsize=(10, 5))

        # 그레이스케일 이미지 출력
        plt.subplot(2, 3, 1)
        plt.title('Grayscale Image')
        plt.imshow(self.gray_image, cmap='gray')
        plt.axis('off')

        # 이진화 이미지 출력
        plt.subplot(2, 3, 2)
        plt.title('Binary Image')
        plt.imshow(self.binary_image, cmap='gray')
        plt.axis('off')

        plt.subplot(2, 3, 3)
        plt.title('Body-only Image')
        plt.imshow(self.body_image, cmap='gray')
        plt.axis('off')

        plt.subplot(2, 3, 4)
        plt.title('Skeleton Image')
        # plt.imshow(self.skeleton, cmap='jet', alpha=0.5)
        plt.imshow(self.body_image, cmap='gray', alpha=1)
        plt.scatter(self.yx_coords[:, 1], self.yx_coords[:, 0], color='blue', s=1, label='Original skeleton')
        # plt.contour(self.longest_backbone_image)
        plt.legend()
        plt.axis('off')

        plt.subplot(2, 3, 5)
        plt.imshow(self.body_image, cmap='gray')
        # plt.scatter(self.extended_curve[:, 1], self.extended_curve[:, 0], color='red', s=10)
        plt.scatter(self.extended_yx_coords[:, 1], self.extended_yx_coords[:, 0], color='orange', s=4, label='Extended skeleton')
        plt.scatter(self.yx_coords[:, 1], self.yx_coords[:, 0], color='blue', s=1, label='Original skeleton')
        plt.legend()
        plt.title('Extended Pixel')
        plt.axis('scaled')

        plt.subplot(2, 3, 6)
        plt.title('Extended skeleton Image')
        plt.imshow(self.body_image, cmap='gray', alpha=1)
        plt.scatter(self.extended_yx_coords[:, 1], self.extended_yx_coords[:, 0], color='orange', s=1,
                    label='Extended skeleton')
        plt.scatter(self.extended_skeleton_origin_yx[:, 1], self.extended_skeleton_origin_yx[:, 0], color='red', s=4,
                    label='joints')
        plt.legend()
        plt.axis('off')

        # results_0.4mm
        # rads = -np.deg2rad(self.joint_angle_degree)
        rads = self.joint_angle + np.pi/2
        print(f'rads = {rads}', flush=True)
        arrow_length = 15
        u = np.cos(rads) * arrow_length
        v = np.sin(rads) * arrow_length
        v = -v
        vector_scale = 1
        plt.figure(figsize=(10,5))
        # plt.subplot(2, 3, 6)
        plt.title('Grayscale Image')
        plt.imshow(self.gray_image, cmap='gray')
        pixel_yx = np.flip(self.joint_yx_pixel, axis=0)
        plt.scatter(pixel_yx[:, 1], pixel_yx[:, 0], color='red', s=4, label='Joint Point')
        plt.quiver(pixel_yx[:, 1], pixel_yx[:, 0],
                   u, v,
                   angles='xy',
                   scale_units='xy',
                   scale=vector_scale,
                   width=0.005,
                   headwidth=2,
                   headlength=2,
                   headaxislength=2,
                   color='blue'
                   )
        plt.axis('off')

        #############################################################
        plt.figure(figsize=(10,5))
        plt.subplot(3, 3, 1)
        plt.scatter(self.xy_coords[:, 0], self.xy_coords[:, 1], color='black', s=1, label='Original data')
        plt.title('Data unit pixel size')
        plt.axis('scaled')
        # plt.axis('equal')
        plt.grid(True)

        plt.subplot(3, 3, 2)
        plt.scatter(self.new_xy_coords[:, 0], self.new_xy_coords[:, 1], color='black', s=1, label='Normalized data')
        plt.title('Translation to origin data')
        plt.axis('scaled')
        plt.grid(True)

        plt.subplot(3, 3, 3)
        plt.scatter(self.norm_xy_coords[:, 0], self.norm_xy_coords[:, 1], color='black', s=1, label='Normalized data')
        plt.title('Normalized data')
        plt.axis('scaled')
        plt.grid(True)

        plt.subplot(3, 3, 4)
        plt.scatter(self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], color='black', s=1)
        plt.title('Transpose XY to YX data')
        plt.axis('scaled')
        plt.grid(True)

        plt.subplot(3, 3, 5)
        plt.scatter(self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], color='black', s=4)
        plt.scatter(self.extended_norm_coords[:, 0], self.fitted_y_poly4d, color='red', s=1)
        text = rf'{self.popt_poly4d[0]:.2}x^4 + {self.popt_poly4d[1]:.2}x^3 + {self.popt_poly4d[2]:.2}x^2 + {self.popt_poly4d[3]:.2}x'
        plt.title(text)
        plt.axis('scaled')
        plt.grid(True)

        #############################################################
        x_range = np.linspace(self.start_point[0], self.end_point[0], 100)

        # 시각화
        plt.figure(figsize=(8, 6))
        plt.plot(self.norm_trans_extended_xy_coords[:, 0], self.norm_trans_extended_xy_coords[:, 1], 'yo', markersize=10,
                 label='Extended Data(Skeleton) Points')
        plt.plot(self.trans_xy_coords[:, 0], self.trans_xy_coords[:, 1], 'bo', markersize=5, label='Origin Data(Skeleton) Points')
        plt.plot(x_range, self.func_poly4d(x_range), 'k-', label='Fitted Curve')
        plt.plot(self.start_point[0], self.start_point[1], 'ro', label=f'Measured Start Point ({self.start_point[0]:.5f}, {self.start_point[1]:.5f})')
        plt.plot([self.end_point[0]], [self.end_point[1]], 'ro', label=f'Measured End Point ({self.end_point[0]:.5f}, {self.end_point[1]:.5f})')
        plt.plot([self.x0], [self.y0], 'gx', label=f'Start Closest Point ({self.x0:.5f}, {self.y0:.5f})')
        plt.plot([self.x1], [self.y1], 'gx', label=f'End Closest Point ({self.x1:.5f}, {self.y1:.5f})')
        plt.plot(self.joint_x, self.joint_y, 'ro', label=f'Joint Point')
        # plt.legend(loc='upper left', bbox_to_anchor=(0, -0.2), ncol=1)
        plt.legend()
        plt.title('Curve Fitting and Closest Point Visualization')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('scaled')
        plt.grid(True)

        #############################################################
        plt.figure(figsize=(8, 6))
        # plt.subplot(1, 3, 2)
        plt.plot(x_range, self.func_poly4d(x_range), 'k-', label='Fitted Curve')
        plt.plot(self.joint_x, self.joint_y, 'ro', label=f'Joint Point')
        plt.plot(self.start_point[0], self.start_point[1], 'bo',
                 label=f'Measured Start Point ({self.start_point[0]:.5f}, {self.start_point[1]:.5f})')
        plt.plot([self.end_point[0]], [self.end_point[1]], 'bo',
                 label=f'Measured End Point ({self.end_point[0]:.5f}, {self.end_point[1]:.5f})')

        # plt.legend(loc='upper right', bbox_to_anchor=(0, -0.2), ncol=1)
        plt.legend()
        plt.title('Curve Fitting and Closest Point Visualization (Invers-Rot-Transform)')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        plt.grid(True)

        #############################################################

        plt.show()

if __name__ == '__main__':
    rbsc = RBSC()
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    rbsc.postprocess(image)
    rbsc.show()

    # img_dir = 'data/2024-08-08 experiment/2024-08-08-13-52-44 nopayload/images'
    img_dir = 'data/2024-08-08 experiment/2024-08-08-13-46-11 upper_init-X/images'
    images = [img for img in os.listdir(img_dir) if img.endswith(".png") or img.endswith(".jpg")]
    images = natsorted(images)

    # 현재 시간 가져오기
    current_time = datetime.now()
    time_str = current_time.strftime("%Y%m%d_%H%M%S")

    dir_name = f'images_with_arrow_{time_str}'
    create_directory(dir_name)
    for image_name in tqdm(images):
        image_path = os.path.join(img_dir, image_name)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        rbsc.postprocess(image)
        rbsc.plot_save(dir_name)

    # sampling time test
    n = 100
    st = time.time()
    for i in tqdm(range(n)):
        # cProfile.run('rbsc.postprocess(image_path)')
        rbsc.postprocess(image)
        # rbsc.plot_save()
        # print(f'num of sequence = {i}')
    et = time.time()
    sampling_freq = n/(et-st)
    print(f'total time : {et-st} / freq = {sampling_freq}', flush=True)

    # cv2.imwrite('image_gray.png', rbsc.binary_image)
    # cv2.imwrite('image_gray_body.png', rbsc.body_image)
    # cv2.imwrite('image_skeleton.png', rbsc.skeleton)
