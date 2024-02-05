import moveit_commander
import rospy
from geometry_msgs.msg import WrenchStamped

# neutral_pose:
#     panda_joint1: -0.017792060227770554
#     panda_joint2: -0.7601235411041661
#     panda_joint3: 0.019782607023391807
#     panda_joint4: -2.342050140544315
#     panda_joint5: 0.029840531355804868
#     panda_joint6: 1.5411935298621688
#     panda_joint7: 0.7534486589746342


class PandaArm():
    def __init__(self):
        moveit_commander.roscpp_initialize("")
        rospy.init_node('arm', anonymous=True)
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")

        self.force = None

        # use rospt.Sub
        rospy.Subsriber("/franka_state_controller/F_ext",
                        WrenchStamped, self._force_callback)

    def _force_callback(self, msg):
        self.force = msg.wrench.force
        self.torque = msg.wrench.torque

    def move_to_neutral(self):
        continue
