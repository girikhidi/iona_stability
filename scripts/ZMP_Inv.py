#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool, Float)
from geometry_msgs import Twist
import math 
from  sensor_msgs.msg import (Imu)
from gopher_ros_clearcore.msg import Position
from oculus_ros.msg import ControllerJoystick
import numpy as np



class ZMP_Inv():
    """
    
    """

    def __init__(
        self      
    ):
        """
        
        """

        # # Private constants:
        self.NODE_NAME='ZMP_simple'
        self.acc_x=0
        self.acc_y=0
        self.Mass_pos_fin=[0,0,0]
        self.Mass_rob=0
        self.Pos_c=0
        self.flag=0
        self.prev_pos=0
        self.counter=0
        self.CONTROLLER_SIDE='left'
        self.MAX_LINEAR_SPEED=0.5
        self.steady_pos=self.Pos_c

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
            'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'dependency_node_name':
                rospy.Subscriber(
                    f'/dependency_node_name/is_initialized',
                    Bool,
                    self.__dependency_name_callback,
                ),
        }

        # # Topic publisher:
        #Change 
        self.__publisher_abs_pos = rospy.Publisher(
            f'{self.NODE_NAME}/Z_abs_pos',
            Float,
            queue_size=1,
        )

        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/ZMP_ACC',
            Float64MultiArray,
            queue_size=1,
        )

        self.__publisher_pos_rel = rospy.Publisher(
            f'{self.NODE_NAME}/Z_pos_rel',
            Float,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            'MyBase/imu1', #change
            Imu,
            self.__IMU_callback,
        )

        rospy.Subscriber(
            'COF_total/COM_Fin', #change
            Float64MultiArray,
            self.__COM_Total_callback,
        )

        rospy.Subscriber(
            'gopher_ros_slearcore/chest_position', #change
            Position,
            self.__Chest_Pos_callback,
        )

        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/joystick',
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
    def __IMU_callback(self, message):
        self.acc_x = message.linear_acceleration[0]
        self.acc_y = message.linear_acceleration[1]

    def __COM_Total_callback(self, message):
        self.Mass_pos_fin=[message.data[0], message.data[1], message.data[2]]
        self.Mass_rob=message.data[3]

    def __Chest_Pos_callback(self, message):
        self.Pos_c=message.position
    
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
    
    def acc_calc(self, target_linear_acc):

        R=math.sqrt(self.Mass_pos_fin[0]*self.Mass_pos_fin[0]+self.Mass_pos_fin[1]*self.Mass_pos_fin[1])
        e=2.7
        XlimLow=-0.221/e
        YlimLow=-0.221/e
        cosa=self.Mass_pos_fin[0]/R
        sina=self.Mass_pos_fin[1]/R
        tga=self.Mass_pos_fin[1]/self.Mass_pos_fin[0]

        g=9.81
        acc_X_max_current=(XlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[0])/(-self.Mass_rob*self.Mass_pos_fin[2])
        acc_Y_max_current=(YlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[1])/(-self.Mass_rob*self.Mass_pos_fin[2])
        a_c_max=(acc_X_max_current-tga*acc_Y_max_current-target_linear_acc)/(-cosa-tga*sina)
        a_t_max=(acc_Y_max_current-sina*a_c_max)/(cosa)
        rot_vel=math.sqrt(a_c_max/R)
        rot_acc=a_t_max/R

        return rot_vel, rot_acc
    
    def target_val(self):
        R=math.sqrt(self.Mass_pos_fin[0]*self.Mass_pos_fin[0]+self.Mass_pos_fin[1]*self.Mass_pos_fin[1])
        e=2.7
        XlimLow=-0.221/e
        g=9.81
        P_gain=1

        cosa=self.Mass_pos_fin[0]/R
        sina=self.Mass_pos_fin[1]/R

        hb=0.359
        hc=0.386

        acc_X_max=(XlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[0])/(-self.Mass_rob*(hb+hc))
        
        updated_joystick = self.__check_dead_zones()

        if abs(self.__oculus_joystick.position_y) > 0.01 and abs(self.__oculus_joystick.position_x) <= 0.01:  # Noisy joystick.
            targe_lin_vel = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                [-self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED],
            )
            target_linear_acc = np.interp(
                round(updated_joystick[1], 4),
                [0, 1.0],
                [0.08*acc_X_max, 0.8*acc_X_max],
            )
            counter=0
            rot_vel, rot_acc=self.acc_calc(target_linear_acc)
            target_rot_vel=0
            target_rot_acc=rot_acc

        elif abs(self.__oculus_joystick.position_y) > 0.01 and abs(self.__oculus_joystick.position_x) > 0.01:
            targe_lin_vel = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                [-self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED],
            )
            target_linear_acc = np.interp(
                round(updated_joystick[1], 4),
                [0, 1.0],
                [0.08*acc_X_max, 0.8*acc_X_max],
            )
            
            counter=0
            rot_vel, rot_acc=self.acc_calc(target_linear_acc)

            target_rot_acc=np.interp(
                round(updated_joystick[0], 4),
                [0, 1.0],
                [0.01*rot_acc, rot_acc],
            )
            
            target_rot_vel = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [-rot_vel, rot_vel],
            )

        elif abs(self.__oculus_joystick.position_y) <= 0.01 and abs(self.__oculus_joystick.position_x) > 0.01:
            
            targe_lin_vel = 0 
            target_linear_acc= 0.8*acc_X_max

            counter=0
            rot_vel, rot_acc=self.acc_calc(0)

            target_rot_acc=np.interp(
                round(updated_joystick[0], 4),
                [0, 1.0],
                [0.01*rot_acc, rot_acc],
            )
            
            target_rot_vel = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [-rot_vel, rot_vel],
            )

        elif abs(self.__oculus_joystick.position_y) < 0.01:
            targe_lin_vel=0
            target_linear_acc=0.8*acc_X_max
            rot_vel, rot_acc=self.acc_calc(0)
            target_rot_vel=0
            target_rot_acc=rot_acc
            counter=counter+1
            if counter >50:
                steady_pos=self.Pos_c

        acc_X_max_current=(XlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[0])/(-self.Mass_rob*self.Mass_pos_fin[2])
        
        a_c=R*target_rot_vel*target_rot_vel
        a_t=R*target_rot_acc
        acc_X_current=-a_c*cosa+a_t*sina+target_linear_acc

        if acc_X_max_current>acc_X_current:
            add_pos=steady_pos-self.Pos_c
        else:
            e_acc=acc_X_max_current-acc_X_current
            add_pos=e_acc*P_gain
  
        #abs_pos_msg = Float()
        add_pos_msg = Float()
        float64_array = Float64MultiArray()
        #abs_pos_msg.data=abs_pos
        add_pos_msg.data=add_pos
        float64_array.data=[target_linear_acc, target_rot_acc, target_rot_vel, targe_lin_vel]

        self.__publisher.publish(float64_array)
        #self.__publisher_abs_pos.publish(abs_pos_msg)
        self.__publisher_pos_rel.publish(add_pos_msg)
    
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.target_val()
        


        
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
    rospy.init_node('ZMP_Inv')

    class_instance = ZMP_Inv()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()