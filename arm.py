import moveit_commander
import rospy
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState
from time import sleep

class PandaArm():
    def __init__(self):
        moveit_commander.roscpp_initialize("")
        rospy.init_node('arm', anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

        self.force = None
        self.torque = None
        self.state = None
        self.z_contact = False
        self.z_collision = False

        self.error_publisher = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal,queue_size=10)
        
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self._force_callback)
        
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self._franka_state_callback)

    def _force_callback(self, msg: WrenchStamped):
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque
    
    def _franka_state_callback(self, msg: FrankaState):
        self.state = msg
        self.z_contact = msg.cartesian_contact[2] > 0
        self.z_collision = msg.cartesian_collision[2] > 0


    def clear_error(self):
        msg = ErrorRecoveryActionGoal()
        self.error_publisher.publish(msg)
        sleep(3)

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

    def set_speed(self, speed):
        self.move_group.set_max_velocity_scaling_factor(speed*2)

    def move_to_cartesian(self, pose, wait=True):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=wait)

    def get_current_pose(self):
        return self.move_group.get_current_pose().pose

    def move_to_contact(self, time):

        pose = self.get_current_pose()
        pose.position.z -= 0.3
        self.move_to_cartesian(pose, wait=False)

        while not self.z_contact:
            print(self.force.z)
            sleep(0.01)
        print(f"Before: Contact {self.z_contact}, Collision {self.z_collision}")
        self.move_group.stop()
        sleep(time)
        print(f"After: Contact {self.z_contact}, Collision {self.z_collision}")
