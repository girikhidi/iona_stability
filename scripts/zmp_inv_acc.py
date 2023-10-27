#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from geometry_msgs.msg import Point
import math
from gopher_ros_clearcore.msg import Position
from oculus_ros.msg import ControllerJoystick
import numpy as np


class ZMP_Inv():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:
        self.NODE_NAME = 'zmp_inv'
        self.acc_x = 0
        self.acc_y = 0
        self.Mass_pos_fin = [0, 0, 0]
        self.Mass_rob = 0
        self.Pos_c = 0
        self.flag = 0
        self.prev_pos = 0
        self.counter = 0
        self.CONTROLLER_SIDE = 'left'
        self.MAX_LINEAR_SPEED = 0.5
        self.steady_pos = self.Pos_c
        self.__oculus_joystick = ControllerJoystick()
        self.__target_linear_acc_init=0

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            # 'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            # 'dependency_node_name':
            #     rospy.Subscriber(
            #         f'/dependency_node_name/is_initialized',
            #         Bool,
            #         self.__dependency_name_callback,
            #     ),
        }

        # # Topic publisher:
        #Change

        self.__publisher = rospy.Publisher(
            '/zmp_acc_max',
            Float64MultiArray,
            queue_size=1,
        )

        self.__publisher_pos_rel = rospy.Publisher(
            '/z_chest_pos',
            Position,
            queue_size=1,
        )

        # # Topic subscriber:

        rospy.Subscriber(
            'calc_com_total/com_fin',  #change
            Float64MultiArray,
            self.__COM_Total_callback,
        )

        rospy.Subscriber(
            'gopher_ros_slearcore/chest_position',  #change
            Position,
            self.__Chest_Pos_callback,
        )

        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )

        rospy.Subscriber(
            '/chest_position',
            Point,
            self.__Chest_Pos_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __COM_Total_callback(self, message):
        self.Mass_pos_fin = [message.data[0], message.data[1], message.data[2]]
        self.Mass_rob = message.data[3]

    def __Chest_Pos_callback(self, message):
        self.Pos_c = message.Z

    def __oculus_joystick_callback(self, message):
        self.__oculus_joystick = message

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:

    def __check_dead_zones(self):
        """
        
        """

        updated_joystick = np.array(
            [
                self.__oculus_joystick.position_x,
                self.__oculus_joystick.position_y,
            ]
        )

        dead_zone_margins = {
            'up': 15,
            'down': 25,
            'left_right': 0,
        }

        angle = math.degrees(
            math.atan2(updated_joystick[1], updated_joystick[0])
        )

        if (
            (
                angle < 90 + dead_zone_margins['up']
                and angle > 90 - dead_zone_margins['up']
            ) or (
                angle > -90 - dead_zone_margins['down']
                and angle < -90 + dead_zone_margins['down']
            )
        ):
            updated_joystick[0] = 0.0

        elif (
            (
                angle < 0 + dead_zone_margins['left_right']
                and angle > 0 - dead_zone_margins['down']
            ) or (
                angle < -180 + dead_zone_margins['left_right']
                and angle > 180 - dead_zone_margins['down']
            )
        ):
            updated_joystick[1] = 0.0

        return updated_joystick

    def max_acc_calc(self, target_linear_acc):

        R=math.sqrt(self.Mass_pos_fin[0]*self.Mass_pos_fin[0]+self.Mass_pos_fin[1]*self.Mass_pos_fin[1])
        e=2.7
        XlimLow=-0.221/e
        YlimLow=-0.221/e
        g=9.81

        cosa=self.Mass_pos_fin[0]/R
        sina=self.Mass_pos_fin[1]/R
        tga=self.Mass_pos_fin[1]/self.Mass_pos_fin[0]
        

        acc_X_max=(XlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[0])/(-self.Mass_rob*self.Mass_pos_fin[2])
       
        acc_Y_max=(YlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[1])/(-self.Mass_rob*self.Mass_pos_fin[2])
      

        a_c_max=(acc_X_max-tga*acc_Y_max-target_linear_acc)/(-cosa-tga*sina)
        a_t_max=(acc_Y_max-sina*a_c_max)/(cosa)
        max_rot_vel=math.sqrt(a_c_max/R)
        max_rot_acc=a_t_max/R

        max_linea_acc=0.8*acc_X_max

        float64_array = Float64MultiArray()
        float64_array.data=[max_linea_acc, max_rot_acc, max_rot_vel]

        self.__publisher.publish(float64_array)

        return max_linea_acc, max_rot_acc, max_rot_vel

    def target_vals_calc(self, MAX_LINEAR_ACCELERATION, MAX_ROTATION_SPEED, MAX_ROTATION_ACCELERATION):

        updated_joystick = self.__check_dead_zones()

        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            # Linear velocity.

            target_linear_acc = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                # NOTE: 0.08 and 0.8 - experimental.
                [-0.8 * MAX_LINEAR_ACCELERATION, 0.8 * MAX_LINEAR_ACCELERATION], 
                )

            if target_linear_velocity <= -0.5 * self.MAX_LINEAR_SPEED:
                target_linear_velocity = -0.5 * self.MAX_LINEAR_SPEED


        if abs(self.__oculus_joystick.position_x) > 0.01:  # Noisy joystick.
            # Rotation velocity.
            target_rotation_velocity = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [MAX_ROTATION_SPEED, -MAX_ROTATION_SPEED],
            )

            target_rotation_acc = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [-MAX_ROTATION_ACCELERATION, MAX_ROTATION_ACCELERATION], 
                )
        return target_linear_acc, target_rotation_velocity, target_rotation_acc
    
    def chest_pos_calc(self, target_linear_acc, target_rot_vel, target_rot_acc):
        R = math.sqrt(
            self.Mass_pos_fin[0] * self.Mass_pos_fin[0]
            + self.Mass_pos_fin[1] * self.Mass_pos_fin[1]
        )
        e = 2.7
        XlimLow = -0.221 / e
        g = 9.81
        P_gain = 1

        cosa = self.Mass_pos_fin[0] / R
        sina = self.Mass_pos_fin[1] / R  

        if abs(self.__oculus_joystick.position_y
                ) < 0.01 and abs(self.__oculus_joystick.position_x) < 0.01:
            counter = counter + 1
            if counter > 50:
                steady_pos = self.Pos_c

        acc_X_max_current = (
            XlimLow * self.Mass_rob * g - self.Mass_rob * self.Mass_pos_fin[0]
        ) / (-self.Mass_rob * self.Mass_pos_fin[2])

        a_c = R * target_rot_vel * target_rot_vel
        a_t = R * target_rot_acc
        acc_X_current = -a_c * cosa + a_t * sina + target_linear_acc

        if acc_X_max_current >= acc_X_current:
            abs_pos = steady_pos
        else:
            e_acc = acc_X_max_current - acc_X_current
            abs_pos = self.Pos_c + e_acc * P_gain

        abs_pos_msg = Position()
        abs_pos_msg.data.position = abs_pos
        abs_pos_msg.data.velocity = 0.5
       
        self.__publisher_pos_rel.publish(abs_pos_msg)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        max_linea_acc, max_rot_acc, max_rot_vel=self.max_acc_calc(self.__target_linear_acc_init)
        self.__target_linear_acc_init, target_rot_vel, target_rot_acc=self.target_vals_calc(max_linea_acc, max_rot_vel, max_rot_acc)
        self.chest_pos_calc(self.__target_linear_acc_init, target_rot_vel, target_rot_acc)
        

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node('zmp_inv')

    class_instance = ZMP_Inv()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()