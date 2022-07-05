import psycopg2 as pg
import rospy

from std_msgs.msg import Int32MultiArray, Float64MultiArray
from modbus.post_threading import Post
from contextlib import closing

# ------------------------------------------------------------------------------------
shares, in_ports, out_ports = [], [], [0] * 106


######################################################################################

##########################################
#           ROS and FT methods           #
##########################################

class BaseApi:

    def __init__(self):
        self.post = Post(self)

    ######################################
    #          Main FT methods           #
    ######################################

    @staticmethod
    def print(msg: str):
        """Print ROS' message
    :param msg: message to print
    :type msg: str
        """
        rospy.loginfo(msg)

    @staticmethod
    def error(msg: str):
        """Print ROS' error-message
    :param msg: message to print
    :type msg: str
        """
        rospy.logerr(msg)

    ##################################################################################

    @staticmethod
    def sleep(time: float):
        """Sleep timer
    :param time: seconds to sleep
    :type time: float
        """
        rospy.sleep(time)

    @staticmethod
    def wait():
        """Waiting for 'Ctrl-C'
        """
        rospy.spin()

    ##################################################################################

    @staticmethod
    def get(port: int):
        """Get input port value
    :param port: input port number
    :type port: int in range (0, 112)
        """
        # r_print(in_ports)
        # print(f'in_ports: {in_ports}')
        return in_ports[port]

    def send(self, value=None):
        """Send output ports values
    :param value: output ports values to send
    :type value: list in range (112)
        """
        output = self.output

        if value is None:
            output.data = out_ports
        else:
            output.data = value

        self.pub.publish(output)

    def set(self, port, sensor=None, time=None, value=1):
        """Set output port value in a special way
    :param port: output port number
    :type port: int in range (0, 106)
    :param sensor: sensor port number, defaults to None
                   resetting port value after sensor value changed
    :type sensor: int in range (0, 112)
    :param time: seconds to reset port value, defaults to None
    :type time: float
    :param value: value to set, defaults to 1
    :type value: int in range (0, 2)
        """
        global out_ports

        out_ports[port] = value
        self.send(out_ports)

        if value == 0:
            return
        elif sensor:
            value = self.get(sensor)
            while self.get(sensor) == value:
                self.sleep(0.001)
            if time:
                self.sleep(time)
        elif time:
            self.sleep(time)
        else:
            return

        out_ports[port] = 0
        self.send(out_ports)

    ##################################################################################

    @staticmethod
    def execute(query: str, result=False):
        """Execute SQL-query
    :param query: query to execute
    :type query: str
    :param result: result return marker, defaults to False
    :type query: bool
    :return: SQL-query result
    :rtype: list of tuples
        """
        with closing(pg.connect(user='postgres',
                                password='panda',
                                host='localhost',
                                database='postgres')) as conn:
            with conn.cursor() as cursor:
                cursor.execute(query)
                conn.commit()
                if result:
                    return cursor.fetchall()
    
    @staticmethod
    def check_shares():
        """Print wallet shares"""
        print(f'- Wallet shares: {shares}')

    def get_color(self):
        """Get current color from analog sensor"""
        print(f'- Current color: {self.get(0)}')

    ##################################################################################

    ######################################
    #           ROS Subscriber           #
    ######################################

    # noinspection PyMethodParameters
    def __in_ports_update(msg):
        """Service callback-function:
    Read input ports and write them to list
    Make input ports available in app
        """
        global in_ports

        del in_ports[0: len(in_ports)]
        # noinspection PyUnresolvedReferences
        in_ports.extend(msg.data)
    sub = rospy.Subscriber("modbus_wrapper/input",
                           Int32MultiArray,
                           __in_ports_update,
                           queue_size=500)

    # noinspection PyMethodParameters
    def __spectator_update(msg):
        """Service callback-function:
    Read wallet shares and write them to list
    Make wallet shares available in app
        """
        global shares

        del shares[0: len(shares)]
        # noinspection PyUnresolvedReferences
        shares.extend(msg.data)
    spectator = rospy.Subscriber("spectator",
                                 Float64MultiArray,
                                 __spectator_update,
                                 queue_size=500)

    # --------------------------------------------------------------------------------

    ######################################
    #           ROS Publisher            #
    ######################################

    pub = rospy.Publisher("modbus_wrapper/output",
                          Int32MultiArray,
                          queue_size=500)
    output = Int32MultiArray()

# ------------------------------------------------------------------------------------

