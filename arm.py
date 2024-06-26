from copy import deepcopy
from time import sleep

import cv2
import moveit_commander
import numpy as np
import pyrealsense2 as rs2
import rospy
import tf.transformations
from cartesian_impedance_controller.msg import ControllerConfig
from control_msgs.msg import (FollowJointTrajectoryActionFeedback,
                              FollowJointTrajectoryActionGoal,
                              FollowJointTrajectoryActionResult)
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, UnloadController)
from cv_bridge import CvBridge, CvBridgeError
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from franka_msgs.srv import (SetEEFrame, SetForceTorqueCollisionBehavior,
                             SetLoad)
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, WrenchStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from iterativeTimeParameterization import \
    IterativeParabolicTimeParameterization

from .gripper import GripperInterface

DEBUG = True


def rprint(msg):
    if DEBUG:
        rospy.loginfo(msg)


class PandaArm:
    def __init__(self):

        self.T_feature_to_robot = None
        self.O_T_EE = None
        self.force = None
        self.torque = None
        self.state = None
        self.robot_mode = None
        self.contact_state = []
        self.collision_state = []
        self.position_trajectory_feedback = None
        self.position_trajectory_status = None
        self.current_feature_zeropoint = Pose(position=Point(
            x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))

        self.calibrated_collision_state = [0, 0, 0]
        self.calibrated_contact_state = [0, 0, 0]
        self.force_x_moving_avg = []
        self.force_y_moving_avg = []
        self.force_z_moving_avg = []
        self.force_x_avg = None
        self.force_y_avg = None
        self.force_z_avg = None

        moveit_commander.roscpp_initialize("")
        rospy.init_node("arm", anonymous=True, disable_signals=True)
        rospy.on_shutdown(self.clean_shutdown)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group.set_end_effector_link("panda_hand_tcp")

        self.error_publisher = rospy.Publisher(
            "/franka_control/error_recovery/goal",
            ErrorRecoveryActionGoal,
            queue_size=10,
        )
        self.impedance_controller_settings_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/set_config",
            ControllerConfig,
            queue_size=50,
        )
        self.impedance_controller_wrench_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/set_cartesian_wrench",
            WrenchStamped,
            queue_size=10,
        )
        self.trajectory_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=10,
        )
        rospy.Subscriber(
            "/franka_state_controller/F_ext", WrenchStamped, self._force_callback
        )
        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self._franka_state_callback,
        )
        rospy.Subscriber(
            "/position_joint_trajectory_controller/follow_joint_trajectory/feedback",
            FollowJointTrajectoryActionFeedback,
            self._position_controller_trajectory_feedback_callback,
        )
        # rospy.Subscriber("/CartesianImpedance_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, self._impedance_controller_trajectory_feedback_callback)
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            callback=self.convert_depth_image,
            queue_size=1,
        )
        rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            callback=self.get_color_image,
            queue_size=1,
        )
        rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.callback_pointcloud
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            self.imageDepthInfoCallback,
        )

        self.depth_image_cv = None
        self.depth_array = None
        self.center_idx = None
        self.gen = None
        self.color_image = None
        self.intrinsics = None

        self.start_default_controller()
        self.controller_name = "position_joint_trajectory_controller"

        self.clear_error()
        self.gripper = GripperInterface()
        self.gripper.open()
        self.gripper.calibrate()

        self.speed = 0.15
        self.set_speed(self.speed)

        self.stop_controller(self.controller_name)

        self.lower_force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        self.upper_force = [
            40.0,
            40.0,
            40.0,
            45.0,
            45.0,
            45.0,
        ]  # [x*2 for x in self.lower_force]
        self.lower_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        self.upper_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        self.set_force_torque_collision_behavior(
            self.lower_torque, self.upper_torque, self.lower_force, self.upper_force
        )

        self.start_controller(self.controller_name)

        self.clear_error()
        self.calc_T_feature_to_robot()

    def convert_depth_image(self, ros_image):
        """Convert the depth image from ROS format to OpenCV format

        Args:
            ros_image: ROS image message
        """
        bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image_cv = bridge.imgmsg_to_cv2(
                ros_image, desired_encoding="passthrough"
            )
            depth_array = np.array(depth_image_cv, dtype=np.float32)
            center_idx = np.array(depth_array.shape) / 2

            self.depth_image_cv = depth_image_cv
            self.depth_array = depth_array
            self.center_idx = center_idx

        except CvBridgeError as e:
            rprint(e)
        # Convert the depth image to a Numpy array

    def imageDepthInfoCallback(self, cameraInfo):
        """Callback for the camera info

        Args:
            cameraInfo: CameraInfo ros message
        """
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == "plumb_bob":
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == "equidistant":
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            rprint(e)
            return

    def get_color_image(self, ros_image):
        """Get the color image from the camera

        Args:
            ros_image: ROS image message
        """
        bridge = CvBridge()
        try:
            self.color_image = bridge.imgmsg_to_cv2(
                ros_image, desired_encoding="passthrough"
            )
        except CvBridgeError as e:
            rprint(e)

    def get_camera_XYZ(self, x, y):
        """Get the Z value of a pixel in the depth image

        Args:
            x: x coordinate of the pixel
            y: y coordinate of the pixel

        Returns:
            float: Z value of the pixel
        """
        if self.depth_image_cv is not None:
            depth_pixel = self.depth_image_cv[int(y)][int(x)]
            result = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [x, y], depth_pixel
            )
            rprint(result)
            return depth_pixel

    def callback_pointcloud(self, data):
        """Callback for the point cloud data

        Args:
            data: PointCloud2 ros message
        """
        assert isinstance(data, PointCloud2)
        self.gen = point_cloud2.read_points_list(
            data, field_names=("x", "y", "z"))
        sleep(1)
        # print(type(self.gen))

    def get_pointcloud(self, x, y):
        """Get the point cloud data of a pixel

        Args:
            x: x coordinate of the pixel
            y: y coordinate of the pixel
        """
        for p in self.gen:
            if p.x == x and p.y == y:
                print(" x : %.3f  y: %.3f  z: %.3f" % (p.x, p.y, p.z))

    def show_image(self, image, x, y):
        """Show an image with a circle at a specific pixel

        Args:
            image: Image to show
            x: x coordinate of the circle center
            y: y coordinate of the circle center
        """
        img = deepcopy(image)
        depth_pixel = self.depth_image_cv[int(y)][int(x)]
        xyz = "x: " + str(y) + " y: " + str(x) + " z: " + str(depth_pixel)
        cv2.putText(img, xyz, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.circle(img, (int(y), int(x)), 10, (0, 0, 255), -1)
        cv2.imshow("image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def clean_shutdown(self):
        """Stop robot when shutting down"""
        rospy.loginfo("System is shutting down. Stopping robot...")
        self.move_group.stop()

    def _force_callback(self, msg: WrenchStamped):
        """Callback for receiving force and torque data from the robot via the /franka_state_controller/F_ext topic

        Args:
            msg: WrenchStamped message containing force and torque data
        """
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque

        self.force_x_avg = self.moving_average(
            self.force.x, self.force_x_moving_avg)
        self.force_y_avg = self.moving_average(
            self.force.y, self.force_y_moving_avg)
        self.force_z_avg = self.moving_average(
            self.force.z, self.force_z_moving_avg)

    def moving_average(self, reading, readings, max_samples=10):
        """Calculate the moving average of a reading

        Args:
            reading: Reading to calculate the moving average of
            readings: List of previous readings
            max_samples: How many readings to average over. Defaults to 10.

        Returns:
            _description_
        """
        readings.append(reading)
        avg = float(sum(readings)) / max(len(readings), 1)
        # print('current average =', avg)
        # print('readings used for average:', readings)
        if len(readings) == max_samples:
            readings.pop(0)
        # print('readings saved for next time:', readings)
        return avg

    def force_magnitude(self, force_x, force_y, force_z):
        """Calculate the magnitude of a force vector

        Args:
            force_x: X component of the force vector
            force_y: Y component of the force vector
            force_z: Z component of the force vector

        Returns:
            float: Magnitude of the force vector
        """
        return np.sqrt(force_x**2 + force_y**2 + force_z**2)

    def _franka_state_callback(self, msg: FrankaState):
        """Callback for receiving state data from the robot via the /franka_state_controller/franka_states topic

        Args:
            msg: FrankaState message containing state data
        """
        self.state = msg
        self.robot_mode = msg.robot_mode
        self.contact_state = msg.cartesian_contact
        self.collision_state = msg.cartesian_collision
        self.O_T_EE = msg.O_T_EE

    def _position_controller_trajectory_feedback_callback(self, msg: FollowJointTrajectoryActionFeedback):
        """Callback for receiving feedback data from the robot via the /position_joint_trajectory_controller/follow_joint_trajectory/feedback topic

        Args:
            msg: FollowJointTrajectoryActionFeedback message containing feedback data
        """
        # 1 if the trajectory is being executed, 3 if the trajectory is completed
        self.position_trajectory_status = msg.status.status
        self.position_trajectory_feedback = msg.feedback

    def position_controller_trajectory_status(self):
        """Wait for the position controller to finish executing the current trajectory
        """
        while self.position_trajectory_status is None:
            sleep(0.1)

        while self.position_trajectory_status != 3:
            # rprint(self.position_trajectory_status)
            sleep(0.1)
        self.position_trajectory_status = None
        sleep(0.1)

    def get_base_rotation(self):
        """Get the current rotation of the robot in the base frame

        Returns:
            list: List containing the current rotation of the robot in the base frame as [roll, pitch, yaw]
        """
        current_pose = self.get_current_pose()
        return tf.transformations.euler_from_quaternion(
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
        )

    def set_load(
        self,
        m_load: float,
        f_center_load: list,
        load_inertia: list,
    ):
        """Set the load of the robot

        Args:
            m_load: Mass in kg
            f_center_load: Center of mass in meters, given as [x, y, z]
            load_inertia: Inertia matrix in kg*m^2, given as [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz]

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rospy.wait_for_service("/franka_control/set_load")
        try:
            rospy.loginfo("Setting load")
            set_load = rospy.ServiceProxy("/franka_control/set_load", SetLoad)
            resp = set_load(m_load, f_center_load, load_inertia)
            return resp.success
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def set_EE_frame(self, NE_T_EE):
        """Set the end effector frame

        Args:
            NE_T_EE: Transformation matrix from the nominal end effector frame to the end effector frame

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        # norminal end effector to end effector transformation
        rospy.wait_for_service("/franka_control/set_EE_frame")
        try:
            rospy.loginfo("Setting load")
            set_EE_frame = rospy.ServiceProxy(
                "/franka_control/set_EE_frame", SetEEFrame
            )
            resp = set_EE_frame(NE_T_EE)
            return resp.success
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def set_force_torque_collision_behavior(self, lower_torque, upper_torque, lower_force, upper_force):
        """Set the force torque collision behavior of the robot

        Args:
            lower_torque: List containing the lower torque values, thees values will be used to determine if contact has occured
            upper_torque: List containing the upper torque values, thees values will be used to determine if a collision has occured
            lower_force: List containing the lower force values, thees values will be used to determine if contact has occured
            upper_force: List containing the upper force values, thees values will be used to determine if a collision has occured

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rospy.wait_for_service(
            "/franka_control/set_force_torque_collision_behavior")
        try:
            rospy.loginfo("Setting force torque collision behavior 1")
            set_force_torque_collision_behavior = rospy.ServiceProxy(
                "/franka_control/set_force_torque_collision_behavior",
                SetForceTorqueCollisionBehavior,
            )
            rospy.loginfo("Setting force torque collision behavior 2")
            resp = set_force_torque_collision_behavior(
                lower_torque, upper_torque, lower_force, upper_force
            )
            rospy.loginfo("Setting force torque collision behavior 3")
            return resp.success
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def zero_force_sensor(self, calibration_points=20, sleep_time=0.0333):
        """Calibrate the force sensor

        Args:
            calibration_points: How many sensor readings to calibrate over. Defaults to 20.
            sleep_time: How long to wait before each reading when calibrating. Defaults to 0.0333.
        """
        raw_force_points_x = []
        raw_force_points_y = []
        raw_force_points_z = []
        for i in range(calibration_points):
            raw_force_points_x.append(self.force.x)
            raw_force_points_y.append(self.force.y)
            raw_force_points_z.append(self.force.z)
            sleep(sleep_time)
        force_mean_x = sum(raw_force_points_x) / calibration_points
        force_mean_y = sum(raw_force_points_y) / calibration_points
        force_mean_z = sum(raw_force_points_z) / calibration_points

        self.calibrate_force_sensor = [
            force_mean_x, force_mean_y, force_mean_z]

    def calibrate_contact_state(self, zero_force_sensor):
        """Calibrate the threshold values for the contact state

        Args:
            zero_force_sensor: List containing the zero force sensor values
        """
        calibrated_force = [
            self.force.x - zero_force_sensor[0],
            self.force.y - zero_force_sensor[1],
            self.force.z - zero_force_sensor[2],
        ]

        for i in range(len(calibrated_force)):
            if abs(calibrated_force[i]) > self.upper_force[i]:
                self.calibrated_collision_state[i] = 1
                self.calibrated_contact_state[i] = 1
            elif (
                abs(calibrated_force[i]) > self.lower_force[i]
                and abs(calibrated_force[i]) < self.upper_force[i]
            ):
                self.calibrated_collision_state[i] = 0
                self.calibrated_contact_state[i] = 1
            elif abs(calibrated_force[i]) < self.lower_force[i]:
                self.calibrated_collision_state[i] = 0
                self.calibrated_contact_state[i] = 0

        rprint(calibrated_force)
        rprint(self.calibrated_collision_state)
        rprint(self.calibrated_contact_state)

    def get_active_controller(self):
        """Get the name of the currently active controller

        Returns:
            str: Name of the currently active controller
        """
        controllers = self.get_controllers()
        for controller in controllers.controller:
            if controller.state == "running" and controller.name not in [
                "franka_state_controller",
                "gripper_controller",
            ]:
                return controller.name

    def start_default_controller(self):
        """Start the default controller (position_joint_trajectory_controller)

            if the current controller is not the default controller, unload the current controller and load the default controller

        """
        controller_name = self.get_active_controller()
        if controller_name == "position_joint_trajectory_controller":
            self.reload_controller(
                controller_name="CartesianImpedance_trajectory_controller"
            )
            return

        rprint("Current controller: " + str(controller_name))
        rprint("Switching to position_joint_trajectory_controller")

        self.stop_controller(controller_name)
        self.reload_controller(
            controller_name="CartesianImpedance_trajectory_controller")
        self.start_controller("position_joint_trajectory_controller")

    def get_controllers(self) -> 'ListControllers':
        """Get a list of all controllers

        Returns:
            ListControllers: List of all controllers
        """
        rospy.wait_for_service('/controller_manager/list_controllers')
        try:
            list_controllers = rospy.ServiceProxy(
                '/controller_manager/list_controllers', ListControllers)
            resp = list_controllers()
            return resp
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def unload_controller(self, controller_name):
        """Unload a controller

        Args:
            controller_name: Name of the controller to unload

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rprint("Unloading controller")
        rospy.wait_for_service("/controller_manager/unload_controller")
        try:
            unload_controller = rospy.ServiceProxy(
                '/controller_manager/unload_controller', UnloadController)
            resp = unload_controller(name=controller_name)
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def load_controller(self, controller_name):
        """Load a controller

        Args:
            controller_name: Name of the controller to load

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rprint("Loading controller")
        rospy.wait_for_service("/controller_manager/load_controller")
        try:
            load_controller = rospy.ServiceProxy(
                '/controller_manager/load_controller', LoadController)
            resp = load_controller(name=controller_name)
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def reload_controller(self, controller_name):
        """Reload a controller (unload and load the controller)

        Args:
            controller_name: Name of the controller to reload
        """
        self.unload_controller(controller_name)
        self.load_controller(controller_name)

    def stop_controller(self, controller_name):
        """Stop a controller

        Args:
            controller_name: Name of the controller to stop

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rprint("Stopping controller")
        if type(controller_name) == str:
            controller_name = [controller_name]
        rospy.wait_for_service("/controller_manager/switch_controller")
        try:
            switch_controller = rospy.ServiceProxy(
                "/controller_manager/switch_controller", SwitchController
            )
            resp = switch_controller(
                start_controllers=[],
                stop_controllers=controller_name,
                strictness=1,
                start_asap=False,
                timeout=0.0,
            )
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def start_controller(self, controller_name):
        """Start a controller

        Args:
            controller_name: Name of the controller to start

        Returns:
            bool: True if the service call was successful, False otherwise
        """
        rprint("Starting controller")
        rospy.wait_for_service("/controller_manager/switch_controller")
        try:
            switch_controller = rospy.ServiceProxy(
                "/controller_manager/switch_controller", SwitchController
            )
            resp = switch_controller(
                start_controllers=[controller_name],
                stop_controllers=[],
                strictness=1,
                start_asap=False,
                timeout=0.0,
            )
            self.controller_name = controller_name
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def set_impedance_controller_trajectory(self, trajectory):
        """Set the trajectory for the impedance controller

        Args:
            trajectory: Trajectory to set for the impedance controller
        """
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory = trajectory
        self.trajectory_publisher.publish(msg)

    def set_impedance_controller_settings(self, stiffness, damping):
        """Set the impedance controller settings

        Args:
            stiffness: List containing the stiffness values for the impedance controller
            damping: List containing the damping values for the impedance controller
        """
        msg = ControllerConfig()
        cart_stiffness = Wrench()
        cart_stiffness.force.x = stiffness[0]
        cart_stiffness.force.y = stiffness[1]
        cart_stiffness.force.z = stiffness[2]
        cart_stiffness.torque.x = stiffness[3]
        cart_stiffness.torque.y = stiffness[4]
        cart_stiffness.torque.z = stiffness[5]

        cart_damping = Wrench()
        cart_damping.force.x = damping[0]
        cart_damping.force.y = damping[1]
        cart_damping.force.z = damping[2]
        cart_damping.torque.x = damping[3]
        cart_damping.torque.y = damping[4]
        cart_damping.torque.z = damping[5]

        msg.cartesian_stiffness = cart_stiffness
        msg.cartesian_damping_factors = cart_damping
        msg.nullspace_stiffness = 30.0
        msg.nullspace_damping_factor = 1.0
        self.impedance_controller_settings_publisher.publish(msg)

    def set_cartestion_impedance_wrench(self, force, torque):
        """Set the wrench for the impedance controller

        Args:
            force: List containing the force
            torque: List containing the torque
        """
        msg = WrenchStamped()
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        self.impedance_controller_wrench_publisher.publish(msg)

    def start_cartestion_impedance_controller(self, stiffness, damping):
        """Start the CartesianImpedance_trajectory_controller

        Args:
            stiffness: List containing the stiffness values for the impedance controller
            damping: List containing the damping values for the impedance controller
        """
        ok_stop = self.stop_controller("position_joint_trajectory_controller")
        if ok_stop:
            ok_start = self.start_controller(
                "CartesianImpedance_trajectory_controller")
            if ok_start:
                self.set_impedance_controller_settings(stiffness, damping)
                rprint("done setting")
            else:
                rprint("Could not start CartesianImpedance_trajectory_controller")
        else:
            rprint("Could not stop position_joint_trajectory_controller")

    def stop_cartestion_impedance_controller(self):
        """Stop the CartesianImpedance_trajectory_controller and start the position_joint_trajectory_controller
        """
        ok_stop = self.stop_controller(
            "CartesianImpedance_trajectory_controller")
        self.reload_controller(
            controller_name="CartesianImpedance_trajectory_controller"
        )
        if ok_stop:
            ok_start = self.start_controller(
                "position_joint_trajectory_controller")
            if ok_start:
                rprint("Started position_joint_trajectory_controller")
            else:
                rprint("Could not start CartesianImpedance_trajectory_controller")
        else:
            rprint("Could not stop position_joint_trajectory_controller")

    def clear_error(self, force=False):
        """Clear the error on the robot

        Args:
            force: If true will clear error even when none are present. Defaults to False.
        """
        sleep(0.5)
        if self.robot_mode in [1, 4] or force:
            msg = ErrorRecoveryActionGoal()
            self.error_publisher.publish(msg)
            rprint("Error cleared")

            while self.robot_mode != 2:
                sleep(0.1)

    def set_current_feature_zeropoint(self, zero_point: Pose):
        """Set the current feature zero point from where all robot poses are calculated

        Args:
            zero_point: Pose of the feature zero point
        """
        self.current_feature_zeropoint = zero_point

    def pose_to_transformation_matrix(self, pose: Pose):
        """Convert a Pose message to a transformation matrix

        Args:
            pose: Pose message to convert

        Returns:
            np.array: Transformation matrix
        """
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        R = tf.transformations.quaternion_matrix(orientation)[:3, :3]
        p = position
        return np.vstack([np.hstack([R, p.reshape(-1, 1)]), [0, 0, 0, 1]])

    def transformation_matrix_to_pose(self, T):
        """Convert a transformation matrix to a Pose message

        Args:
            T: Transformation matrix to convert

        Returns:
            Pose: Pose message
        """
        position = T[:3, 3]
        orientation = tf.transformations.quaternion_from_matrix(T)
        return Pose(position=Point(*position), orientation=Quaternion(*orientation))

    def calc_T_feature_to_robot(self):
        """Calculate the transformation matrix from the feature frame to the robot frame
        """
        pose_wrt_robot = self.current_feature_zeropoint
        pose_wrt_feature = Pose(position=Point(
            x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))

        T_robot = self.pose_to_transformation_matrix(pose_wrt_robot)
        T_feature = self.pose_to_transformation_matrix(pose_wrt_feature)

        # T_FR = T_R @ inv(T_F)
        self.T_feature_to_robot = T_robot @ np.linalg.inv(T_feature)

    def pose_robot_from_pose_feature(self, pose_feature: Pose):
        """Convert a pose from the feature frame to the robot frame

        Args:
            pose_feature: Pose in the feature frame

        Returns:
            Pose: Pose in the robot frame
        """
        T_feature = self.pose_to_transformation_matrix(pose_feature)
        # T_R = T_FR @ T_F
        T_robot = self.T_feature_to_robot @ T_feature
        pose_robot = self.transformation_matrix_to_pose(T_robot)
        return pose_robot

    def pose_feature_from_pose_robot(self, pose_robot: Pose):
        """Convert a pose from the robot frame to the feature frame

        Args:
            pose_robot: Pose in the robot frame

        Returns:
            Pose: Pose in the feature frame
        """
        T_robot = self.pose_to_transformation_matrix(pose_robot)
        # T_F = inv(T_FR) @ T_R
        T_feature = np.linalg.inv(self.T_feature_to_robot) @ T_robot
        pose_feature = self.transformation_matrix_to_pose(T_feature)
        return pose_feature

    def align_to_base(self, x=True, y=True, z=False):
        """Align the robot to the base frame

        Args:
            x: Whether to align the robot in the x-axis. Defaults to True.
            y: Whether to align the robot in the y-axis. Defaults to True.
            z: Whether to align the robot in the z-axis. Defaults to False.
        """
        current_rotation_from_base = self.get_base_rotation()
        self.rotate_abs(
            np.pi if x else current_rotation_from_base[0],
            0 if y else current_rotation_from_base[1],
            0 if z else current_rotation_from_base[2],
        )

    def move_to_neutral(self):
        """Move the robot to the neutral pose
        """
        # neutral_pose:
        #     panda_joint1: -0.017792060227770554
        #     panda_joint2: -0.7601235411041661
        #     panda_joint3: 0.019782607023391807
        #     panda_joint4: -2.342050140544315
        #     panda_joint5: 0.029840531355804868
        #     panda_joint6: 1.5411935298621688
        #     panda_joint7: 0.7534486589746342
        joint_goal = [0, 0, 0, 0, 0, 0, 0]
        joint_goal[0] = -0.017792060227770554
        joint_goal[1] = -0.7601235411041661
        joint_goal[2] = 0.019782607023391807
        joint_goal[3] = -2.342050140544315
        joint_goal[4] = 0.029840531355804868
        joint_goal[5] = 1.5411935298621688
        joint_goal[6] = 0.7534486589746342

        self.move_to_joint(joint_goal)

    def get_speed(self):
        """Get the current speed of the robot"""
        return self.speed

    def set_speed(self, speed):
        """Set the speed of the robot

        Args:
            speed: Speed to set
        """
        self.speed = speed
        self.move_group.set_max_velocity_scaling_factor(self.speed)

    def move_to_joint(self, pose_feature, wait=False):
        """Move the robot to a joint pose

        Args:
            pose_feature: Pose to move to
            wait: Whether to execute the move synchronously or asynchronously. Defaults to False.
        """
        if isinstance(pose_feature, list):
            rprint(msg="Moving to joint")
            self.move_group.go(pose_feature, wait=wait)
        else:
            pose_robot = self.pose_robot_from_pose_feature(pose_feature)
            self.move_group.set_pose_target(
                pose_robot, end_effector_link="panda_hand_tcp"
            )
            self.move_group.go(wait=wait)

        try:
            self.position_controller_trajectory_status()
        except KeyboardInterrupt:
            self.move_group.stop()
            self.clear_error()

    def move_to_cartesian(self, pose_feature, wait=False, speed=0.15, iterations=100, skip_parameterzation=False, execute=True, make_interuptable=True):
        """Move the robot to a cartesian pose

        Args:
            pose_feature: Pose to move to
            wait: Whether to execute the move synchronously or asynchronously. Defaults to False.
            speed: How fast to move. Defaults to 0.15.
            iterations: How many iterations of time parameterization to run. Defaults to 100.
            skip_parameterzation: Whether to skip time parameterization. Defaults to False.
            execute: If move should begin immediately or not. Defaults to True.
            make_interuptable: If it should be possible to interrupt the movement. Defaults to True.

        Returns:
            RobotTrajectory: A time parameterized RobotTrajectory object using the given speed.
        """
        if isinstance(pose_feature, list):
            updateted_pose_feature = []
            for pose in pose_feature:
                updateted_pose_feature.append(
                    self.pose_robot_from_pose_feature(pose))
        else:
            updateted_pose_feature = self.pose_robot_from_pose_feature(
                pose_feature)

        plan, fraction = self.move_group.compute_cartesian_path(
            (
                [*updateted_pose_feature]
                if isinstance(updateted_pose_feature, list)
                else [updateted_pose_feature]
            ),
            0.01,
            0.0,
        )

        if not skip_parameterzation:
            # set cartesian speed using time parameterization
            iptp = IterativeParabolicTimeParameterization()
            plan = iptp.compute_time_stamps(
                plan, speed, iteration_max=iterations)

        if execute:
            # create switch for controller name
            if self.controller_name == "position_joint_trajectory_controller":
                self.move_group.execute(plan, wait=wait)

                if make_interuptable:
                    try:
                        self.position_controller_trajectory_status()
                    except KeyboardInterrupt:
                        self.move_group.stop()
                        self.clear_error()

            elif self.controller_name == "CartesianImpedance_trajectory_controller":
                self.set_impedance_controller_trajectory(plan.joint_trajectory)
            else:
                rprint("Unknown controller name")
        else:
            return plan

    def get_current_pose(self) -> 'Pose':
        """Get the current pose of the robot

        Returns:
            Pose: Current pose of the robot
        """
        pose_robot = self.move_group.get_current_pose().pose
        return self.pose_feature_from_pose_robot(pose_robot)

    def contact(self):
        """Shortcut for move_to_contact
        """
        self.move_to_contact()

    def move(self, pose):
        """Shortcut for move_to_joint

        Args:
            pose: Pose to move to
        """
        self.move_to_joint(pose)

    def move_to_contact(self, target_pose=None, search_distance=0.3, time=0.5, timeout=10.0, only_in_axis=None, speed=0.015) -> 'tuple[list[float]]':
        """Move the robot to a contact state

        Args:
            target_pose: Pose to move towards. Defaults to None.
            search_distance: How far to search in meters. Defaults to 0.3.
            time: How long to wait at contact state. Defaults to 0.5.
            timeout: how long to search before timeout. Defaults to 10.0.
            only_in_axis: if set to 1, 2 or 3 tells robot to only search in the given axis. Defaults to None.
            speed: How fast to move while searching. Defaults to 0.015.

        Returns:
            tuple[list[float]]: Tuple containing the contact and collision state of the robot
        """
        # AXIS FOLLOWS THE TCP FRAME AND NOT THE BASE FRAME
        if calibrate_force_sensor:
            self.zero_force_sensor()
        calibrated_force = [
            self.force_x_avg - self.calibrate_force_sensor[0],
            self.force_y_avg - self.calibrate_force_sensor[1],
            self.force_z_avg - self.calibrate_force_sensor[2],
        ]

        if force_threshold is not None:
            force_threshold_xyz = [force_threshold,
                                   force_threshold, force_threshold]
        else:
            force_threshold_xyz = self.lower_force

        if not target_pose:
            target_pose = self.get_current_pose()
            target_pose.position.z -= search_distance

        current_speed = self.speed
        self.set_speed(speed)

        self.move_to_cartesian(
            target_pose, wait=False, speed=speed, make_interuptable=False
        )
        rprint("Moving to contact")
        start_time = rospy.get_time()

        try:
            if only_in_axis is not None:
                # and (self.get_current_pose() != target_pose)
                while (
                    abs(calibrated_force[only_in_axis])
                    < force_threshold_xyz[only_in_axis]
                ) and (rospy.get_time() - start_time < timeout):
                    # sleep(0.01)
                    calibrated_force = [
                        self.force_x_avg - self.calibrate_force_sensor[0],
                        self.force_y_avg - self.calibrate_force_sensor[1],
                        self.force_z_avg - self.calibrate_force_sensor[2],
                    ]
                    rprint(calibrated_force)

            else:
                while (
                    (abs(calibrated_force[0]) < force_threshold_xyz[0])
                    and (abs(calibrated_force[1]) < force_threshold_xyz[1])
                    and (abs(calibrated_force[2]) < force_threshold_xyz[2])
                    and (rospy.get_time() - start_time < timeout)
                ):
                    # sleep(0.01)
                    calibrated_force = [
                        self.force_x_avg - self.calibrate_force_sensor[0],
                        self.force_y_avg - self.calibrate_force_sensor[1],
                        self.force_z_avg - self.calibrate_force_sensor[2],
                    ]
                    rprint(calibrated_force)
            self.move_group.stop()
            sleep(time)
            self.set_speed(current_speed)
            rprint("DONE")

            if rospy.get_time() - start_time >= timeout:
                rprint("Timeout")
                return False
            else:
                rprint("Contact made")
                rprint(calibrated_force)
                return True

        except KeyboardInterrupt:
            self.move_group.stop()
            sleep(time)
            self.clear_error()
            self.set_speed(current_speed)

        end_state = (self.contact_state, self.collision_state)

        self.move_group.stop()
        sleep(time)

        self.set_speed(current_speed)

        return end_state

    def relative_move(self, axis: int, distance: float):
        """Move the robot relative to the current position

        Args:
            axis: Axis to move along
            distance: Distance to move

        Returns:
            Pose: Pose of the robot after moving
        """
        if axis not in [0, 1, 2]:
            rprint("Invalid axis")
            return

        pose = self.get_current_pose()
        if axis == 0:
            pose.position.x += distance
        elif axis == 1:
            pose.position.y += distance
        else:
            pose.position.z += distance

        self.move_to_cartesian(pose, speed=speed)

        return pose

    def rotate(self, x, y, z, move=True, direction=True):
        """Rotate the robot relative to the current orientation

        Args:
            x: How much to rotate around the x-axis
            y: How much to rotate around the y-axis
            z: How much to rotate around the z-axis
            move: Whether the move should be executed immediately. Defaults to True.
            direction: If set will allow robot to reverse the previous rotation. Defaults to True.

        Returns:
            _description_
        """
        q_r = tf.transformations.quaternion_from_euler(x, y, z)

        current_orientation = self.get_current_pose().orientation

        if direction:
            quaternion = tf.transformations.quaternion_multiply(
                q_r,
                [
                    current_orientation.x,
                    current_orientation.y,
                    current_orientation.z,
                    current_orientation.w,
                ],
            )
        else:
            q_r = tf.transformations.quaternion_conjugate(q_r)
            quaternion = tf.transformations.quaternion_multiply(
                q_r,
                [
                    current_orientation.x,
                    current_orientation.y,
                    current_orientation.z,
                    current_orientation.w,
                ],
            )

        ori = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
        )

        target_pose = self.get_current_pose()
        target_pose.orientation = ori

        if move:
            self.move_to_cartesian(target_pose)

        return target_pose

    def rotate_abs(self, x, y, z, move=True):
        """Rotate the robot to an absolute orientation

        Args:
            x: Where to rotate around the x-axis
            y: Where to rotate around the y-axis
            z: Where to rotate around the z-axis
            move: Whether the move should be executed immediately. Defaults to True.

        Returns:
            Pose: Pose of the robot after rotating
        """
        target_quaternion = tf.transformations.quaternion_from_euler(x, y, z)
        ori = Quaternion(
            x=target_quaternion[0],
            y=target_quaternion[1],
            z=target_quaternion[2],
            w=target_quaternion[3],
        )

        target_pose = self.get_current_pose()
        target_pose.orientation = ori

        if move:
            self.move_to_cartesian(target_pose)

        return target_pose
