import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2


class CameraController(Node):
    def __init__(self, track_target = False, hit_target = False, analysis_results = None, node_name='camera_controller'):
        super().__init__(node_name)
        qos_profile_unity = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.analysis_results = analysis_results

        self.target_pose = None
        self.target_pose_pixels = None
        self.target_pose_angles = None
        self.target_pose_angles_gt = None

        self.fov_horizontal = None
        self.fov_vertical  = None
        self.image_width = None
        self.image_height = None
        self.focus_distance_x = None
        self.focus_distance_y = None

        self.camera_pose = None
        self.camera_orientation = None
        self.camera_orientation_angles = None

        self.track_target = track_target
        self.hit_target = hit_target
        #self.camera_orientation_angles_init = [0, np.deg2rad(-46), 0]  # Ros frame
        self.camera_orientation_angles_init = [np.deg2rad(10), np.deg2rad(20), np.deg2rad(30)]

        self.t = 0
        self.dt = 0
        self.num_frame = 0

        self.camera_pose_publisher = self.create_publisher(
            PoseStamped,
            '/camera_pose_in',
            qos_profile_unity
        )

        print(f"publish init {self.camera_orientation_angles_init}")
        self.publish_camera_pose(camera_pose=[0, 0, 0], camera_angles=self.camera_orientation_angles_init)

    def get_focus_distance(self, fov, length):
        return length / (2 * np.tan(np.deg2rad(fov / 2)))

    def init_parameter(self, measurement):

        self.num_frame += 1
        self.t = measurement["time"]
        self.dt = measurement["dt"]

        (self.fov_vertical, self.fov_horizontal,
        camera_pose_x, camera_pose_y, camera_pose_z,
        camera_orientation_x, camera_orientation_y, camera_orientation_z, camera_orientation_w,
        target_pose_x, target_pose_y, target_pose_z) = measurement["unity_payload"]

        self.camera_pose = camera_pose_x, camera_pose_y, camera_pose_z
        self.camera_orientation = camera_orientation_x, camera_orientation_y, camera_orientation_z, camera_orientation_w
        #print(f"self.camera_orientation = {self.camera_orientation}")
        self.camera_orientation_angles = self.quaternion_to_euler(self.camera_orientation)
        #print(f"self.camera_orientation_angles = {self.camera_orientation_angles}")

        self.target_pose = target_pose_x, target_pose_y, target_pose_z

        self.image_width = measurement["image_width"]
        self.image_height = measurement["image_height"]
        self.focus_distance_x = self.get_focus_distance(self.fov_horizontal, self.image_width)
        self.focus_distance_y = self.get_focus_distance(self.fov_vertical, self.image_height)

        #print(f"focus_distance_x = {self.focus_distance_x} focus_distance_y = {self.focus_distance_y}")

    def track_drone(self, measurement):

        gray = cv2.cvtColor(measurement["frame"].copy(), cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)
        gradient = cv2.convertScaleAbs(gradient)

        if measurement["centroid"] == (None, None):
            return

        self.init_parameter(measurement)
        x, y = measurement["centroid"]
        target_x = int(round(x * self.image_width))
        target_y = int(round(y * self.image_height))

        good_tracker = True
        # check if the target is within the image boundaries
        # if the target is too close to the image boundaries, set good_tracker to False
        boundary_threshold = 20
        if (target_x < boundary_threshold or target_x > measurement["frame"].shape[1] - boundary_threshold or
                target_y < boundary_threshold or target_y > measurement["frame"].shape[0] - boundary_threshold):
            good_tracker = False
            self.get_logger().info(f'Target out of bounds: {target_x}, {target_y}')

        data_valid = self.camera_orientation_angles is not None
        if good_tracker:
            self.target_pose_pixels = (target_x, target_y) # (x, y) coordinates
            self.target_pose_angles = self.convert_pixels_to_angles(target_x, target_y)

            #print(f"target_x = {target_x}, target_y = {target_y}")
            #print(f"theta = {self.target_pose_angles[0]}, psi = {self.target_pose_angles[1]}")
            x_pix, y_pix = self.convert_angles_to_pixels(self.target_pose_angles[0], self.target_pose_angles[1])
            #print(f"x_pix = {x_pix}, y_pix = {y_pix}")

            self.analysis_with_target_pose()

            self.draw_gt_point_on_image(gradient)

            if self.track_target and data_valid:
                dt_step = 0.5
                theta = self.camera_orientation_angles[1] + dt_step * self.target_pose_angles[0]
                psi = self.camera_orientation_angles[2] - dt_step * self.target_pose_angles[1]
                if self.num_frame % 2 == 0:
                    self.publish_camera_pose(camera_angles=[0, theta, psi])

            if self.hit_target and data_valid:
                dt_step = 0.5
                theta = self.camera_orientation_angles[1] + dt_step * self.target_pose_angles[0]
                psi = self.camera_orientation_angles[2] - dt_step * self.target_pose_angles[1]

                cam_pose = self.get_camera_pose_to_target() # чому направляємо по минулій орієнтації камери, а не майбутньої, яку вирахували?
                self.get_logger().info(f'Camera pose: {cam_pose}')
                if cam_pose is not None and self.num_frame % 3 == 0:
                    self.publish_camera_pose(camera_pose=cam_pose, camera_angles=[0, theta, psi])

            cv2.circle(gradient, self.target_pose_pixels, 15, (255, 0, 0))
        else:
            cv2.rectangle(gradient, (target_x - 10, target_y - 10), (target_x + 10, target_y + 10), (0, 0, 255), 2)
        # Display the gradient image
        cv2.imshow('Gradient Image', gradient)

        cv2.waitKey(1)

    def convert_pixels_to_angles(self, x, y):
        # Convert pixel coordinates to angles
        theta = np.arctan2(y - self.image_height / 2, self.focus_distance_y)
        psi = np.arctan2(x - self.image_width / 2, self.focus_distance_x)
        return theta, psi

    def convert_angles_to_pixels(self, theta, psi):
        x_pixel = int(round(self.focus_distance_x * np.tan(psi) + self.image_width / 2))
        y_pixel = int(round(self.focus_distance_y * np.tan(theta) + self.image_height / 2))
        return x_pixel, y_pixel

    def draw_gt_point_on_image(self, frame):
        theta_gt, psi_gt = self.target_pose_angles_gt
        x_pixel = int(round(self.focus_distance_x * np.tan(psi_gt) + self.image_width / 2))
        y_pixel = int(round(self.focus_distance_y * np.tan(theta_gt) + self.image_height / 2))

        if 0 <= x_pixel < self.image_width and 0 <= y_pixel < self.image_height:
            cv2.circle(frame, (x_pixel, y_pixel), 8, (0, 0, 255), 2)
            #self.get_logger().info(f'Drew GT point at: x={x_pixel}, y={y_pixel}')
        else:
            pass
            #self.get_logger().info(f'GT point out of image bounds: x={x_pixel}, y={y_pixel}')

    def quaternion_to_euler(self, orientation):
        # Convert quaternion to Euler angles (roll, pitch, yaw) using scipy
        r = R.from_quat(orientation) # [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles (roll, pitch, yaw) to quaternion using scipy
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        qx, qy, qz, qw = r.as_quat()  # scipy returns [x, y, z, w]
        return qx, qy, qz, qw

    def get_camera_pose_to_target(self):
        # local x axis of a camera points to center of the image
        v_camera = 10 # m/s
        r = R.from_quat(self.camera_orientation)
        rotation_matrix = r.as_matrix()
        # x axis of the camera frame in the world frame (direction to fly)
        e_x = rotation_matrix @ np.array([1, 0, 0])
        cam_pose = self.camera_pose + e_x * v_camera * self.dt
        return cam_pose

    def rotate_point_to_camera_frame(self, point, orientation):
        # Convert quaternion to rotation matrix
        r = R.from_quat(orientation)
        rotation_matrix = r.as_matrix()
        point_camera_frame = np.dot(rotation_matrix.T, np.array(point))
        return point_camera_frame

    def analysis_with_target_pose(self):

        if self.camera_orientation_angles is None:
            # self.get_logger().info('Camera orientation angles not available yet.')
            return

        # global frame
        # rotate the target pose to the camera frame using self.camera_orientation
        target_pose_camera_frame = self.rotate_point_to_camera_frame(self.target_pose, self.camera_orientation)
        t_x, t_y, t_z = target_pose_camera_frame
        target_elevation_angle = - np.arctan2(t_z, np.sqrt(t_x ** 2 + t_y ** 2))
        target_azimuth_angle = - np.arctan2(t_y, t_x)
        #print(f"target_elevation_angle: {target_elevation_angle} target_azimuth_angle = {target_azimuth_angle}")
        self.target_pose_angles_gt = [target_elevation_angle, target_azimuth_angle] # relative to the camera frame
        self.analysis_results.log_camera_control(
            time=self.t,
            theta=self.target_pose_angles[0],
            psi=self.target_pose_angles[1],
            theta_gt=self.target_pose_angles_gt[0],
            psi_gt=self.target_pose_angles_gt[1],
        )

    def publish_camera_pose(self, camera_pose=None, camera_angles=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'  # або як назвете свій frame

        # --- позиція ---
        if camera_pose is None:
            camera_pose = (0.0, 0.0, 0.0)
        msg.pose.position.x = float(camera_pose[0])
        msg.pose.position.y = float(camera_pose[1])
        msg.pose.position.z = float(camera_pose[2])

        # --- орієнтація ---
        if camera_angles is None:
            camera_angles = self.camera_orientation_angles_init
        roll, pitch, yaw = camera_angles
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # публікуємо
        self.camera_pose_publisher.publish(msg)
        self.get_logger().info(
            f'Published camera pose: pos={camera_pose}, quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})'
        )

    def publish_camera_pose_old(self, camera_pose=None, camera_angles=None):
            # camera angles in rad. in ros frame coordinates
            msg = Float32MultiArray()
            if camera_pose is None:
                camera_pose = (0.0, 0.0, 0.0)
            if camera_angles is None:
                camera_angles = self.camera_orientation_angles_init

            roll, theta, psi = camera_angles[0], camera_angles[1], camera_angles[2]
            qx, qy, qz, qw = self.euler_to_quaternion(roll, theta, psi)
            msg.data = [
                float(camera_pose[0]), float(camera_pose[1]), float(camera_pose[2]),
                qx, qy, qz, qw
            ]
            self.camera_pose_publisher.publish(msg)
            self.get_logger().info(f'Published camera_pose: {camera_pose}, qx, qy, qz, qw = {qx, qy, qz, qw}')
            # self.get_logger().info(f'Camera angles published: theta={theta}, psi={psi}')
