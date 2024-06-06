import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class IterativeParabolicTimeParameterization:
    def __init__(self):
        pass

    def compute_time_stamps(self, plan: RobotTrajectory, speed, iteration_max=100):

        num_points = len(plan.joint_trajectory.points)
        time_diff = [0.0] * (num_points - 1)
        time_diff, plan = self.apply_velocity_constraints(
            plan, time_diff, speed)
        time_diff, plan = self.apply_acceleration_constraints(
            plan, time_diff, speed, iteration_max)
        plan = self.update_trajectory(plan, time_diff)

        return plan

    def apply_velocity_constraints(self, plan: RobotTrajectory, time_diff, speed):
        for i in range(len(plan.joint_trajectory.points)-1):
            current_point: JointTrajectoryPoint = plan.joint_trajectory.points[i]
            next_point: JointTrajectoryPoint = plan.joint_trajectory.points[i+1]

            vars = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                    'panda_joint6', 'panda_joint7']
            max_velocitys = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]

            joint_limits_base = "/robot_description_planning/joint_limits/"
            for idx, v in enumerate(vars):
                v_max = max_velocitys[idx] * speed

                dq1 = current_point.positions[idx]
                dq2 = next_point.positions[idx]
                t_min = abs(dq2 - dq1) / v_max
                if t_min > time_diff[i]:
                    time_diff[i] = t_min

        return time_diff, plan

    def findT1(self, dq1, dq2, dt1, dt2, a_max):
        mult_factor = 1.01
        v1 = dq1 / dt1
        v2 = dq2 / dt2
        a = 2.0 * (v2 - v1) / (dt1 + dt2)

        while abs(a) > a_max:
            v1 = dq1 / dt1
            v2 = dq2 / dt2
            a = 2.0 * (v2 - v1) / (dt1 + dt2)
            dt1 *= mult_factor

        return dt1

    def findT2(self, dq1, dq2, dt1, dt2, a_max):
        mult_factor = 1.01
        v1 = dq1 / dt1
        v2 = dq2 / dt2
        a = 2.0 * (v2 - v1) / (dt1 + dt2)

        while abs(a) > a_max:
            v1 = dq1 / dt1
            v2 = dq2 / dt2
            a = 2.0 * (v2 - v1) / (dt1 + dt2)
            dt2 *= mult_factor

        return dt2

    def apply_acceleration_constraints(self, plan: RobotTrajectory, time_diff, speed, iteration_max=100):
        num_updates = 0
        iteration = 0
        backwards = False
        num_points = len(plan.joint_trajectory.points)
        vars = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                'panda_joint6', 'panda_joint7']
        max_accelerations = [3.75, 1.875, 2.5, 3.125, 3.75, 5, 5]

        joint_limits_base = "/robot_description_planning/joint_limits/"

        while True:
            num_updates = 0
            iteration += 1

            for idx, v in enumerate(vars):
                for count in range(2):
                    for i in range(num_points-1):
                        index = (num_points - 1)-i if backwards else i
                        curr_point: JointTrajectoryPoint = plan.joint_trajectory.points[index]

                        if index > 0:
                            prev_point: JointTrajectoryPoint = plan.joint_trajectory.points[index-1]

                        if index < num_points-1:
                            next_point: JointTrajectoryPoint = plan.joint_trajectory.points[index+1]

                        a_max = max_accelerations[idx] * speed

                        if index == 0:
                            q1 = next_point.positions[idx]
                            q2 = curr_point.positions[idx]
                            q3 = next_point.positions[idx]

                            dt1 = dt2 = time_diff[index]
                            assert not backwards

                        elif index < num_points-1:
                            q1 = prev_point.positions[idx]
                            q2 = curr_point.positions[idx]
                            q3 = next_point.positions[idx]

                            dt1 = time_diff[index-1]
                            dt2 = time_diff[index]

                        else:
                            q1 = prev_point.positions[idx]
                            q2 = curr_point.positions[idx]
                            q3 = prev_point.positions[idx]

                            dt1 = dt2 = time_diff[index-1]
                            assert backwards

                        if dt1 == 0.0 or dt2 == 0.0:
                            v1 = 0.0
                            v2 = 0.0
                            a = 0.0
                        else:
                            start_velocity = False
                            if index == 0:
                                if curr_point.velocities != []:
                                    start_velocity = True
                                    v1 = curr_point.velocities[idx]

                            v1 = v1 if start_velocity else (q2 - q1) / dt1
                            v2 = (q3 - q2) / dt2
                            a = 2.0 * (v2 - v1) / (dt1 + dt2)

                        if abs(a) > a_max + 0.1:
                            if not backwards:
                                dt2 = min(
                                    dt2 + 0.01, self.findT2(q2-q1, q3-q2, dt1, dt2, a_max))
                                time_diff[index] = dt2
                            else:
                                dt1 = min(dt1+0.01, self.findT1(q2 -
                                          q1, q3-q2, dt1, dt2, a_max))
                                time_diff[index-1] = dt1

                            num_updates += 1

                            if dt1 == 0.0 or dt2 == 0.0:
                                v1 = 0.0
                                v2 = 0.0
                                a = 0.0
                            else:
                                v1 = (q2-q1)/dt1
                                v2 = (q3-q2)/dt2
                                a = 2.0 * (v2 - v1) / (dt1 + dt2)

                backwards = not backwards

            if not ((num_updates > 0) and (iteration < iteration_max)):
                break

        return time_diff, plan

    def update_trajectory(self, plan: RobotTrajectory, time_diff):
        time_sum = 0.0
        num_points = len(plan.joint_trajectory.points)
        vars = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                'panda_joint6', 'panda_joint7']

        plan.joint_trajectory.points[0].time_from_start = rospy.Duration(0.0)

        # Times
        for i in range(1, num_points):
            # update the time betwwen each point
            time_sum += time_diff[i-1]
            plan.joint_trajectory.points[i].time_from_start = rospy.Duration(
                time_sum)

        # Return if there is only one point
        if num_points <= 1:
            return plan

        # Accelerations
        for i in range(num_points):
            current_point: JointTrajectoryPoint = plan.joint_trajectory.points[i]

            if i > 0:
                prev_point: JointTrajectoryPoint = plan.joint_trajectory.points[i-1]

            if i < num_points-1:
                next_point: JointTrajectoryPoint = plan.joint_trajectory.points[i+1]

            for idx, v in enumerate(vars):
                if i == 0:
                    q1 = next_point.positions[idx]
                    q2 = current_point.positions[idx]
                    q3 = q1

                    dt1 = dt2 = time_diff[i]
                elif i < num_points-1:
                    q1 = prev_point.positions[idx]
                    q2 = current_point.positions[idx]
                    q3 = next_point.positions[idx]

                    dt1 = time_diff[i-1]
                    dt2 = time_diff[i]
                else:
                    q1 = prev_point.positions[idx]
                    q2 = current_point.positions[idx]
                    q3 = q1

                    dt1 = dt2 = time_diff[i-1]

                start_velocity = False
                if dt1 == 0.0 or dt2 == 0.0:
                    v1 = 0.0
                    v2 = 0.0
                    a = 0.0
                else:
                    if i == 0:
                        if current_point.velocities != []:
                            start_velocity = True
                            v1 = current_point.velocities[idx]

                    v1 = v1 if start_velocity else (q2 - q1) / dt1
                    v2 = v1 if start_velocity else (q3 - q2) / dt2
                    a = 2.0 * (v2 - v1) / (dt1 + dt2)

                current_vels = list(current_point.velocities)
                current_vels[idx] = (v2+v1)/2

                current_accs = list(current_point.accelerations)
                current_accs[idx] = a

                current_point.velocities = current_vels
                current_point.accelerations = current_accs

        return plan
