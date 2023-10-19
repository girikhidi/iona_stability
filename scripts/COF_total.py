#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from geometry_msgs.msg import Point
from math import cos
from math import sin
import numpy as np


class Cof_total():
    """
    
    """

    def __init__(
        self 
    ):
        """
        
        """

        # # Private constants:
        _m_mobile=68
        _m_stand=39.107
        _m_base=_m_mobile+_m_stand
        _m_chest=17.316
        _hb=0.359
        _hc=0.386 # height of the origin of the cheat coordinate system
        _hs=0.56219 # height of center of mass of a stand
        _xr=0.1375 # x transition of the robot in the frame of a chest
        _xl=0.1375 # x transition of the robot in the frame of a chest
        _yr=0.139 # y transition of the robot in the frame of a chest
        _yl=-0.139 # y transition of the robot in the frame of a chest
        _zr=0.1925 # z transition of the robot in the frame of a chest
        _zl=0.1925 # z transition of the robot in the frame of a chest
        _ar=45*np.pi/180
        _al=45*np.pi/180
        _off_chest=0.0995

        self.NODE_NAME='calc_com_total'
        self.Mass_pos_robot_R=[0.0, 0.0, 0.0]
        self.Mass_pos_robot_L=[0.0, 0.0, 0.0]
        self.Mass_pos_fin=[0.0, 0.0, 0.0]
        self.Mass_L=0
        self.Mass_R=0
        self.Pos_c=0

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
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/com_fin',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            'calc_com_arm_l/com_arm_l', #change
            Float64MultiArray,
            self.__COM_arm_L_callback,
        )

        rospy.Subscriber(
            'calc_com_arm_r/com_arm_r', #change
            Float64MultiArray,
            self.__COM_arm_R_callback,
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
    def __COM_arm_L_callback(self, message):
        data = message.data
        self.Mass_pos_robot_L=[data[0], data[1], data[2]]
        self.Mass_L=data[3]

    def __COM_arm_R_callback(self, message):
        data = message.data
        self.Mass_pos_robot_R=[data[0], data[1], data[2]]
        self.Mass_R=data[3]
        
    def __Chest_Pos_callback(self, message):
        self.Pos_c=message.Z
        

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
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        m_mobile=Cof_total._m_mobile
        m_stand=Cof_total._m_stand
        m_base=Cof_total._m_base
        m_chest=Cof_total._m_chest
        hb=Cof_total._hb
        hc=Cof_total._hc
        hs=Cof_total._hs
        xr=Cof_total._xr
        xl=Cof_total._xl
        yr=Cof_total._yr
        yl=Cof_total._yl
        zr=Cof_total._zr
        zl=Cof_total._zl
        ar=Cof_total._ar
        al=Cof_total._al
        off_chest=Cof_total._off_chest

        Mass_pos_robotR=np.array([[self.Mass_pos_robot_R[0]], [self.Mass_pos_robot_R[1]], [self.Mass_pos_robot_R[2]], [1]])
        Mass_pos_robotL=np.array([[self.Mass_pos_robot_L[0]], [self.Mass_pos_robot_L[1]], [self.Mass_pos_robot_L[2]], [1]])
        Mass_pos_mob=np.array([[0.0024], [0], [0.180], [1]]);
        Mass_pos_stand=np.array([[-0.06945], [-0.0016], [hb+hs], [1]])
        Xb=(Mass_pos_mob[0]*m_mobile+Mass_pos_stand[0]*m_stand)/(m_base)
        Yb=(Mass_pos_mob[1]*m_mobile+Mass_pos_stand[1]*m_stand)/(m_base)
        Zb=(Mass_pos_mob[2]*m_mobile+Mass_pos_stand[2]*m_stand)/(m_base)
        Mass_pos_base=np.array([[Xb[0]], [Yb[0]], [Zb[0]], [1]])
        
        T01=np.array([[1, 0, 0, -off_chest],
        [0, 1, 0, 0],
        [0, 0, 1, (hb+hc+self.Pos_c)],
        [0, 0, 0, 1]])
        Mass_pos_chest_1=np.array([[0.08581], [0], [0.172112], [1]])
        Mass_pos_chest=np.dot(T01, Mass_pos_chest_1)

        RotZR=np.array([[0,1,0,0],
            [-1,0,0,0],
            [0,0,1,0],
            [0,0,0,1]])
        TransXR=np.array([[1,0,0,xr],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])
        TransYR=np.array([[1,0,0,0],
            [0,1,0,yr],
            [0,0,1,0],
            [0,0,0,1]])
        TransZR=np.array([[1,0,0,0],
            [0,1,0,0],
            [0,0,1,zr],
            [0,0,0,1]])
        RotYR=np.array([[cos(ar), 0, sin(ar), 0],
            [0, 1,0,0],
            [-sin(ar), 0, cos(ar), 0],
            [0,0,0,1]])

        Mass_pos_robotR=np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T01,RotZR),TransXR),TransYR),TransZR),RotYR),Mass_pos_robotR)

        RotZL=np.array([[0,-1,0,0],
            [1,0,0,0],
            [0,0,1,0],
            [0,0,0,1]])
        TransXL=np.array([[1,0,0,xl],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
        TransYL=np.array([[1,0,0,0],
                [0,1,0,yl],
                [0,0,1,0],
                [0,0,0,1]])
        TransZL=np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,zl],
                [0,0,0,1]])
        RotYL=np.array([[cos(al), 0, sin(al), 0],
                [0, 1,0,0],
                [-sin(al), 0, cos(al), 0],
                [0,0,0,1]])
        Mass_pos_robotL=np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T01,RotZL),TransXL),TransYL),TransZL),RotYL),Mass_pos_robotL)

        Xf=(Mass_pos_base[0]*m_base+Mass_pos_chest[0]*m_chest+
        Mass_pos_robotR[0]*self.Mass_R+Mass_pos_robotL[0]*self.Mass_L)/(self.Mass_R+self.Mass_L+m_chest+m_base)
        Yf=(Mass_pos_base[1]*m_base+Mass_pos_chest[1]*m_chest+
        Mass_pos_robotR[1]*self.Mass_R+Mass_pos_robotL[1]*self.Mass_L)/(self.Mass_R+self.Mass_L+m_chest+m_base)
        Zf=(Mass_pos_base[2]*m_base+Mass_pos_chest[2]*m_chest+
        Mass_pos_robotR[2]*self.Mass_R+Mass_pos_robotL[2]*self.Mass_L)/(self.Mass_R+self.Mass_L+m_chest+m_base)
       
        M_total=self.Mass_R+self.Mass_L+m_chest+m_base
        Mass_pos_fin=[Xf[0], Yf[0], Zf[0], M_total]
        float64_array = Float64MultiArray()
        float64_array.data=Mass_pos_fin

        self.__publisher.publish(float64_array)

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
    rospy.init_node('calc_com_total')

    class_instance = Cof_total()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()