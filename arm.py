import moveit_commander
import rospy
from geometry_msgs.msg import WrenchStamped, Pose, Quaternion
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from franka_msgs.srv import SetForceTorqueCollisionBehavior
from time import sleep
from .gripper import GripperInterface
import tf.transformations
import numpy as np

DEBUG = True
def rprint(msg):
    if DEBUG:
        rospy.loginfo(msg)

def eulerToQuaternion(current_orientation:Quaternion, x, y, z):
    q2=tf.transformations.quaternion_from_euler(x,y,z)

    quaternion=tf.transformations.quaternion_multiply(q2,[current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

    # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    ori = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    return ori

def quaternionToEuler(quaternion: Quaternion):
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w] )
    
    return euler

class PandaArm():
    def __init__(self):
        moveit_commander.roscpp_initialize("")
        rospy.init_node('arm', anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

        self.error_publisher = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal,queue_size=10)
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self._force_callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self._franka_state_callback)

        self.clear_error()
        self.gripper = GripperInterface()
        self.gripper.calibrate()

        self.speed = 0.15
        self.set_speed(self.speed)

        self.O_T_EE = None
        self.force = None
        self.torque = None
        self.state = None
        self.contact_state = []
        self.collision_state = []

        self.lower_force = [5.0, 5.0, 5.0, 6.0, 6.0, 6.0]
        self.upper_force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        self.lower_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        self.upper_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]  
        self.set_force_torque_collision_behavior(self.lower_torque, self.upper_torque, self.lower_force, self.upper_force)

    def _force_callback(self, msg: WrenchStamped):
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque

    def _franka_state_callback(self, msg: FrankaState):
        self.state = msg
        self.contact_state = msg.cartesian_contact
        self.collision_state = msg.cartesian_collision
        self.O_T_EE = msg.O_T_EE

    def get_base_rotation(self):
        arr = np.array(self.O_T_EE)
        reshaped_arr = arr.reshape(4,4)
        rotation = reshaped_arr[:3,:3]
        return tf.transformations.euler_from_matrix(rotation)

    def set_force_torque_collision_behavior(self, lower_torque, upper_torque, lower_force, upper_force):
        rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
        try:
            rospy.loginfo("Setting force torque collision behavior 1")
            set_force_torque_collision_behavior = rospy.ServiceProxy('/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
            rospy.loginfo("Setting force torque collision behavior 2")
            resp = set_force_torque_collision_behavior(lower_torque, upper_torque, lower_force, upper_force)
            rospy.loginfo("Setting force torque collision behavior 3")
            return resp.success
        except rospy.ServiceException as e:
            rprint("Service call failed: %s"%e)

    def clear_error(self):
        msg = ErrorRecoveryActionGoal()
        self.error_publisher.publish(msg)
        sleep(3)

    def align_to_base(self, x=True, y=True, z=False):
        rprint("Aligning to base")
        current_roatation_from_base = self.get_base_rotation()
        rprint(f"Current rotation from base: {current_roatation_from_base}")
        self.rotate(-(np.pi-current_roatation_from_base[0]) if x else 0, -(current_roatation_from_base[1]) if y else 0, -(current_roatation_from_base[2]) if z else 0)
    
    def move_to_neutral(self):
        # neutral_pose:
        #     panda_joint1: -0.017792060227770554
        #     panda_joint2: -0.7601235411041661
        #     panda_joint3: 0.019782607023391807
        #     panda_joint4: -2.342050140544315
        #     panda_joint5: 0.029840531355804868
        #     panda_joint6: 1.5411935298621688
        #     panda_joint7: 0.7534486589746342
        joint_goal = [0,0,0,0,0,0,0]
        joint_goal[0] = -0.017792060227770554
        joint_goal[1] = -0.7601235411041661
        joint_goal[2] = 0.019782607023391807
        joint_goal[3] = -2.342050140544315
        joint_goal[4] = 0.029840531355804868
        joint_goal[5] = 1.5411935298621688
        joint_goal[6] = 0.7534486589746342

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def get_speed(self):
        return self.speed

    def set_speed(self, speed):
        self.speed = speed
        self.move_group.set_max_velocity_scaling_factor(self.speed)

    def get_speed(self)->'float':
        return self.speed

    def move_to_cartesian(self, pose, wait=True):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=wait)

    def get_current_pose(self)->'Pose':
        return self.move_group.get_current_pose().pose

    def contact(self):
        self.move_to_contact()

    def move(self, pose):   
        self.move_to_cartesian(pose)


    def move_to_contact(self, target_pose=None, search_distance=0.3, time=0.5, timeout=10.0) -> 'tuple[list[float]]':

        if not target_pose:
            target_pose = self.get_current_pose()
            target_pose.position.z -= search_distance


        current_speed = self.speed
        slow_speed = 0.02
        self.set_speed(slow_speed)

        self.move_to_cartesian(target_pose, wait=False)

        start_time = rospy.get_time()

        current_contact_state = self.contact_state

        while (1 not in current_contact_state) and (rospy.get_time() - start_time < timeout):
            # rprint(self.force.z)
            current_contact_state = self.contact_state
            sleep(0.01)
        
        end_state = (self.contact_state, self.collision_state)

        rprint(f"Before: Contact {self.contact_state}, Collision {self.collision_state}")
        self.move_group.stop()
        sleep(time)
        rprint(f"After: Contact {self.contact_state}, Collision {self.collision_state}")

        self.set_speed(current_speed)

        return end_state
        # rprint("after ", self.speed) 

    def relative_move(self, axis:int, distance: float):
        if axis not in [0,1,2]:
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

    def rotate(self, x, y, z, move=True):
        q_r=tf.transformations.quaternion_from_euler(x,y,z)
        
        current_orientation = self.get_current_pose().orientation

        quaternion=tf.transformations.quaternion_multiply(q_r,[current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        ori = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        if move:
            target_orientation = self.get_current_pose()
            target_orientation.orientation = ori
            self.move_to_cartesian(target_orientation)

        return ori


