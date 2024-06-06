from time import sleep

import moveit_commander
import numpy as np
import rospy
import tf.transformations
from cartesian_impedance_controller.msg import ControllerConfig
from control_msgs.msg import (FollowJointTrajectoryActionFeedback,
                              FollowJointTrajectoryActionGoal,
                              FollowJointTrajectoryActionResult)
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, UnloadController)
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from franka_msgs.srv import SetForceTorqueCollisionBehavior
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, WrenchStamped

from iterativeTimeParameterization import \
    IterativeParabolicTimeParameterization

from .gripper import GripperInterface

DEBUG = True


def rprint(msg):
    if DEBUG:
        rospy.loginfo(msg)


class PandaArm():
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

        moveit_commander.roscpp_initialize("")
        rospy.init_node('arm', anonymous=True, disable_signals=True)
        rospy.on_shutdown(self.clean_shutdown)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group.set_end_effector_link("panda_hand_tcp")

        self.error_publisher = rospy.Publisher(
            "/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=10)
        self.impedance_controller_settings_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/set_config", ControllerConfig, queue_size=50)
        self.impedance_controller_wrench_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/set_cartesian_wrench", WrenchStamped, queue_size=10)
        self.trajectory_publisher = rospy.Publisher(
            "/CartesianImpedance_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        rospy.Subscriber("/franka_state_controller/F_ext",
                         WrenchStamped, self._force_callback)
        rospy.Subscriber("/franka_state_controller/franka_states",
                         FrankaState, self._franka_state_callback)
        rospy.Subscriber("/position_joint_trajectory_controller/follow_joint_trajectory/feedback",
                         FollowJointTrajectoryActionFeedback, self._position_controller_trajectory_feedback_callback)
        self.start_default_controller()
        self.controller_name = "position_joint_trajectory_controller"

        self.clear_error()
        self.gripper = GripperInterface()
        self.gripper.open()
        self.gripper.calibrate()

        self.speed = 0.15
        self.set_speed(self.speed)

        self.stop_controller(self.controller_name)

        self.lower_force = [10.0, 10.0, 10.0, 13.0, 13.0, 13.0]
        # [x*2 for x in self.lower_force]
        self.upper_force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        self.lower_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        self.upper_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        self.set_force_torque_collision_behavior(
            self.lower_torque, self.upper_torque, self.lower_force, self.upper_force)

        self.start_controller(self.controller_name)

        self.clear_error()
        self.calc_T_feature_to_robot()

    def clean_shutdown(self):
        ''' Stop robot when shutting down '''
        rospy.loginfo("System is shutting down. Stopping robot...")
        self.move_group.stop()

    def _force_callback(self, msg: WrenchStamped):
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque

    def _franka_state_callback(self, msg: FrankaState):
        self.state = msg
        self.robot_mode = msg.robot_mode
        self.contact_state = msg.cartesian_contact
        self.collision_state = msg.cartesian_collision
        self.O_T_EE = msg.O_T_EE

    def _position_controller_trajectory_feedback_callback(self, msg: FollowJointTrajectoryActionFeedback):
        # 1 if the trajectory is being executed, 3 if the trajectory is completed
        self.position_trajectory_status = msg.status.status
        self.position_trajectory_feedback = msg.feedback

    def position_controller_trajectory_status(self):
        while self.position_trajectory_status is None:
            sleep(0.1)

        while self.position_trajectory_status != 3:
            # rprint(self.position_trajectory_status)
            sleep(0.1)
        self.position_trajectory_status = None
        sleep(0.1)

    def get_base_rotation(self):
        current_pose = self.get_current_pose()
        return tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])

    def set_force_torque_collision_behavior(self, lower_torque, upper_torque, lower_force, upper_force):
        rospy.wait_for_service(
            '/franka_control/set_force_torque_collision_behavior')
        try:
            rospy.loginfo("Setting force torque collision behavior 1")
            set_force_torque_collision_behavior = rospy.ServiceProxy(
                '/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
            rospy.loginfo("Setting force torque collision behavior 2")
            resp = set_force_torque_collision_behavior(
                lower_torque, upper_torque, lower_force, upper_force)
            rospy.loginfo("Setting force torque collision behavior 3")
            return resp.success
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def get_active_controller(self):
        controllers = self.get_controllers()
        for controller in controllers.controller:
            if controller.state == "running" and controller.name not in ["franka_state_controller", "gripper_controller"]:
                return controller.name

    def start_default_controller(self):
        controller_name = self.get_active_controller()
        if controller_name == "position_joint_trajectory_controller":
            self.reload_controller(
                controller_name="CartesianImpedance_trajectory_controller")
            return

        rprint("Current controller: " + str(controller_name))
        rprint("Switching to position_joint_trajectory_controller")

        self.stop_controller(controller_name)
        self.reload_controller(
            controller_name="CartesianImpedance_trajectory_controller")
        self.start_controller("position_joint_trajectory_controller")

    def get_controllers(self) -> 'ListControllers':
        rospy.wait_for_service('/controller_manager/list_controllers')
        try:
            list_controllers = rospy.ServiceProxy(
                '/controller_manager/list_controllers', ListControllers)
            resp = list_controllers()
            return resp
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def unload_controller(self, controller_name):
        rprint("Unloading controller")
        rospy.wait_for_service('/controller_manager/unload_controller')
        try:
            unload_controller = rospy.ServiceProxy(
                '/controller_manager/unload_controller', UnloadController)
            resp = unload_controller(name=controller_name)
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def load_controller(self, controller_name):
        rprint("Loading controller")
        rospy.wait_for_service('/controller_manager/load_controller')
        try:
            load_controller = rospy.ServiceProxy(
                '/controller_manager/load_controller', LoadController)
            resp = load_controller(name=controller_name)
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def reload_controller(self, controller_name):
        self.unload_controller(controller_name)
        self.load_controller(controller_name)

    def stop_controller(self, controller_name):
        rprint("Stopping controller")
        if type(controller_name) == str:
            controller_name = [controller_name]
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy(
                '/controller_manager/switch_controller', SwitchController)
            resp = switch_controller(start_controllers=[
            ], stop_controllers=controller_name, strictness=1, start_asap=False, timeout=0.0)
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def start_controller(self, controller_name):
        rprint("Starting controller")
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy(
                '/controller_manager/switch_controller', SwitchController)
            resp = switch_controller(start_controllers=[controller_name], stop_controllers=[
            ], strictness=1, start_asap=False, timeout=0.0)
            self.controller_name = controller_name
            return resp.ok
        except rospy.ServiceException as e:
            rprint("Service call failed: %s" % e)

    def set_impedance_controller_trajectory(self, trajectory):
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory = trajectory
        self.trajectory_publisher.publish(msg)

    def set_impedance_controller_settings(self, stiffness, damping):
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
        msg = WrenchStamped()
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        self.impedance_controller_wrench_publisher.publish(msg)

    def start_cartestion_impedance_controller(self, stiffness, damping):
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

        ok_stop = self.stop_controller(
            "CartesianImpedance_trajectory_controller")
        self.reload_controller(
            controller_name="CartesianImpedance_trajectory_controller")
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

        sleep(0.5)
        if self.robot_mode in [1, 4] or force:
            msg = ErrorRecoveryActionGoal()
            self.error_publisher.publish(msg)
            rprint("Error cleared")

            while self.robot_mode != 2:
                sleep(0.1)

    def pose_to_transformation_matrix(self, pose: Pose):
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        R = tf.transformations.quaternion_matrix(orientation)[:3, :3]
        p = position
        return np.vstack([np.hstack([R, p.reshape(-1, 1)]), [0, 0, 0, 1]])

    def transformation_matrix_to_pose(self, T):
        position = T[:3, 3]
        orientation = tf.transformations.quaternion_from_matrix(T)
        return Pose(position=Point(*position), orientation=Quaternion(*orientation))

    def calc_T_feature_to_robot(self):
        # position:
        #     x: 0.38034927530988794
        #     y: -0.3282111268132951
        #     z: 0.03519540893130724
        # orientation:
        #     x: 0.9999806260596
        #     y: -0.002497528511030583
        #     z: 0.004649514434723419
        #     w: 0.0033002836708662677

        pose_wrt_robot = Pose(position=Point(x=0.38034927530988794, y=-0.3282111268132951, z=0.03519540893130724),
                              orientation=Quaternion(x=0.9999806260596, y=-0.002497528511030583, z=0.004649514434723419, w=0.0033002836708662677))
        pose_wrt_feature = Pose(position=Point(
            x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))

        T_robot = self.pose_to_transformation_matrix(pose_wrt_robot)
        T_feature = self.pose_to_transformation_matrix(pose_wrt_feature)

        # T_FR = T_R @ inv(T_F)
        self.T_feature_to_robot = T_robot @ np.linalg.inv(T_feature)

    def pose_robot_from_pose_feature(self, pose_feature: Pose):
        T_feature = self.pose_to_transformation_matrix(pose_feature)
        # T_R = T_FR @ T_F
        T_robot = self.T_feature_to_robot @ T_feature
        pose_robot = self.transformation_matrix_to_pose(T_robot)
        return pose_robot

    def pose_feature_from_pose_robot(self, pose_robot: Pose):
        T_robot = self.pose_to_transformation_matrix(pose_robot)
        # T_F = inv(T_FR) @ T_R
        T_feature = np.linalg.inv(self.T_feature_to_robot) @ T_robot
        pose_feature = self.transformation_matrix_to_pose(T_feature)
        return pose_feature

    def align_to_base(self, x=True, y=True, z=False):
        current_rotation_from_base = self.get_base_rotation()
        self.rotate_abs(
            np.pi if x else current_rotation_from_base[0], 0 if y else current_rotation_from_base[1], 0 if z else current_rotation_from_base[2])

    def move_to_neutral(self):
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
        return self.speed

    def set_speed(self, speed):
        self.speed = speed
        self.move_group.set_max_velocity_scaling_factor(self.speed)

    def get_speed(self) -> 'float':
        return self.speed

    def move_to_joint(self, pose_feature, wait=False):
        if isinstance(pose_feature, list):
            rprint(msg="Moving to joint")
            self.move_group.go(pose_feature, wait=wait)
        else:
            pose_robot = self.pose_robot_from_pose_feature(pose_feature)
            self.move_group.set_pose_target(
                pose_robot, end_effector_link="panda_hand_tcp")
            self.move_group.go(wait=wait)

        try:
            self.position_controller_trajectory_status()
        except KeyboardInterrupt:
            self.move_group.stop()
            self.clear_error()

    def move_to_cartesian(self, pose_feature, wait=False, speed=0.15, iterations=100, skip_parameterzation=False, execute=True, make_interuptable=True):
        if isinstance(pose_feature, list):
            updateted_pose_feature = []
            for pose in pose_feature:
                updateted_pose_feature.append(
                    self.pose_robot_from_pose_feature(pose))
        else:
            updateted_pose_feature = self.pose_robot_from_pose_feature(
                pose_feature)

        plan, fraction = self.move_group.compute_cartesian_path(
            [*updateted_pose_feature] if isinstance(updateted_pose_feature, list) else [updateted_pose_feature], 0.01, 0.0)

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
        pose_robot = self.move_group.get_current_pose().pose
        return self.pose_feature_from_pose_robot(pose_robot)

    def contact(self):
        self.move_to_contact()

    def move(self, pose):
        self.move_to_joint(pose)

    def move_to_contact(self, target_pose=None, search_distance=0.3, time=0.5, timeout=10.0, only_in_axis=None, speed=0.015) -> 'tuple[list[float]]':
        # AXIS FOLLOWS THE TCP FRAME AND NOT THE BASE FRAME
        if not target_pose:
            target_pose = self.get_current_pose()
            target_pose.position.z -= search_distance

        current_speed = self.speed
        self.set_speed(speed)

        self.move_to_cartesian(target_pose, wait=False,
                               speed=speed, make_interuptable=False)

        start_time = rospy.get_time()

        current_contact_state = self.contact_state

        try:
            if only_in_axis is not None:
                while (current_contact_state[only_in_axis] == 0.0) and (rospy.get_time() - start_time < timeout):
                    current_contact_state = self.contact_state
                    sleep(0.01)
            else:
                while (1 not in current_contact_state) and (rospy.get_time() - start_time < timeout):
                    current_contact_state = self.contact_state
                    sleep(0.01)
        except KeyboardInterrupt:
            self.move_group.stop()
            self.clear_error()

        end_state = (self.contact_state, self.collision_state)

        self.move_group.stop()
        sleep(time)

        self.set_speed(current_speed)

        return end_state

    def relative_move(self, axis: int, distance: float):
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

        self.move_to_cartesian(pose)

        return pose

    def rotate(self, x, y, z, move=True, direction=True):
        q_r = tf.transformations.quaternion_from_euler(x, y, z)

        current_orientation = self.get_current_pose().orientation

        if direction:
            quaternion = tf.transformations.quaternion_multiply(
                q_r, [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        else:
            q_r = tf.transformations.quaternion_conjugate(q_r)
            quaternion = tf.transformations.quaternion_multiply(
                q_r, [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        ori = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        target_pose = self.get_current_pose()
        target_pose.orientation = ori

        if move:
            self.move_to_cartesian(target_pose)

        return target_pose

    def rotate_abs(self, x, y, z, move=True):
        target_quaternion = tf.transformations.quaternion_from_euler(x, y, z)
        ori = Quaternion(x=target_quaternion[0], y=target_quaternion[1],
                         z=target_quaternion[2], w=target_quaternion[3])

        target_pose = self.get_current_pose()
        target_pose.orientation = ori

        if move:
            self.move_to_cartesian(target_pose)

        return target_pose
