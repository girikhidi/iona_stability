#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from geometry_msgs.msg import Point
import math
from gopher_ros_clearcore.msg import Position
import numpy as np
from oculus_ros.msg import (
    ControllerButtons,
    ControllerJoystick,
)


class ZMP_Inv():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:
        self.NODE_NAME = 'zmp_inv'
        self.MAX_CHEST_VELOCITY=0.5
        self.__mass_pos_fin = [0, 0, 0]
        self.__mass_rob = 0
        self.__pos_c = 0
        self.__counter = 0
        self.__steady_pos = self.__pos_c
        self.__abs_pos_c=self.__pos_c
        self.__target_linear_acc_init=0
        self.__current_linear_acc=0
        self.__current_rotational_acc=0
        self.__target_rotational_velocity=0
        self.__target_linear_velocity=0
        self.__p_gain=1
        self.__oculus_joystick = ControllerJoystick()
        self.__joystick_button_state = 0
        self.__control_mode = 'autonomy_control'

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
            '/chest_position',
            Point,
            self.__Chest_Pos_callback,
        )

        rospy.Subscriber(
            '/current_acc',
            Float64MultiArray,
            self.__current_acc_callback,
        )

        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __COM_Total_callback(self, message):
        self.__mass_pos_fin = [message.data[0], message.data[1], message.data[2]]
        self.__mass_rob = message.data[3]

    def __Chest_Pos_callback(self, message):
        self.__pos_c = message.Z

    def __current_acc_callback(self, message):
        self.__current_linear_acc=message.data[0]
        self.__current_rotational_acc=message.data[1]
        self.__target_linear_velocity=message.data[2]
        self.__current_rotational_acc=message.data[3]

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
    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__control_mode = 'user_control'
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 2

        # State 2: Joystick button was pressed. Normal control mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 2
        ):
            self.__control_mode = 'autonomy_control'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 3
        ):
            self.__joystick_button_state = 0

    def max_acc_calc(self, current_linear_acc):

        R=math.sqrt(self.__mass_pos_fin[0]*self.__mass_pos_fin[0]+self.__mass_pos_fin[1]*self.__mass_pos_fin[1])
        e=2.7
        XlimLow=-0.221/e
        YlimLow=-0.221/e
        g=9.81

        cosa=self.__mass_pos_fin[0]/R
        sina=self.__mass_pos_fin[1]/R
        tga=self.__mass_pos_fin[1]/self.__mass_pos_fin[0]
        

        acc_X_max=(XlimLow*self.__mass_rob*g-self.__mass_rob*self.__mass_pos_fin[0])/(-self.__mass_rob*self.__mass_pos_fin[2])
       
        acc_Y_max=(YlimLow*self.__mass_rob*g-self.__mass_rob*self.__mass_pos_fin[1])/(-self.__mass_rob*self.__mass_pos_fin[2])
      

        a_c_max=(acc_X_max-tga*acc_Y_max-current_linear_acc)/(-cosa-tga*sina)
        a_t_max=(acc_Y_max-sina*a_c_max)/(cosa)
        max_rot_vel=math.sqrt(a_c_max/R)
        max_rot_acc=a_t_max/R

        max_linea_acc=0.8*acc_X_max

        float64_array = Float64MultiArray()
        float64_array.data=[max_linea_acc, max_rot_acc, max_rot_vel]

        self.__publisher.publish(float64_array)

    
    def chest_pos_calc(self, current_linear_acc, target_rot_vel, current_rot_acc):
        R = math.sqrt(
            self.__mass_pos_fin[0] * self.__mass_pos_fin[0]
            + self.__mass_pos_fin[1] * self.__mass_pos_fin[1]
        )
        e = 2.7
        XlimLow = -0.221 / e
        g = 9.81
        

        cosa = self.__mass_pos_fin[0] / R
        sina = self.__mass_pos_fin[1] / R  

        # if self.__target_linear_velocity==0 and self.__target_rotational_velocity==0: 
        #     self.__counter = self.__counter + 1
        #     if self.__counter > 50:
        #         self.__steady_pos = self.__pos_c
        # else:
        #     self.__counter=0

        acc_X_max_current = (
            XlimLow * self.__mass_rob * g - self.__mass_rob * self.__mass_pos_fin[0]
        ) / (-self.__mass_rob * self.__mass_pos_fin[2])

        a_c = R * target_rot_vel * target_rot_vel
        a_t = R * current_rot_acc
        acc_X_current = -a_c * cosa + a_t * sina + current_linear_acc
        
        #Position controller
        e_acc = acc_X_max_current - acc_X_current
        self.__abs_pos_c = self.__pos_c + e_acc * self.__p_gain

        if self.__abs_pos_c>=440:
            self.__abs_pos_c=440

    def __publish_pos(self):

        abs_pos_msg = Position()
        abs_pos_msg.data.velocity = self.MAX_CHEST_VELOCITY
        
        self.__joystick_button_state_machine()

        if self.__control_mode == 'user_control':
           abs_pos_msg.data.position = self.__steady_pos
        else:
            abs_pos_msg.data.position = self.__abs_pos_c
        self.__publisher_pos_rel.publish(abs_pos_msg)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.max_acc_calc(self.__current_linear_acc)

        self.chest_pos_calc(self.__target_linear_acc_init, self.__target_rotational_velocity, self.__current_rotational_acc)

        self.__publish_pos()
        
        print(self.__control_mode)
        

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