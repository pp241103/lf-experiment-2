import psycopg2 as pg
import math as m
import rospy

from std_msgs.msg import Int32MultiArray, Float64MultiArray
from inspect import getmembers, isfunction
from modbus.process import ProcessWrapper
from modbus.post_threading import Post
from contextlib import closing

##########################################
#           ROS and FT methods           #
##########################################

shares = []
in_ports = []
out_ports = [0] * 106


class Rapi:
    # noinspection PyUnresolvedReferences
    from modbus_plc_siemens import algorithms
    locals().update(dict((k, v) for (k, v) in getmembers(algorithms, isfunction)))
    del algorithms

    def __init__(self):
        self.post = Post(self)

    ######################################
    #          Main FT methods           #
    ######################################

    @staticmethod
    def print(msg):
        """Print ROS' message
    :param msg: message to print
    :type msg: str
        """
        rospy.loginfo(msg)

    @staticmethod
    def error(msg):
        """Print ROS' error-message
    :param msg: message to print
    :type msg: str
        """
        rospy.logerr(msg)

    #######################################################

    @staticmethod
    def sleep(time):
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

    #######################################################

    @staticmethod
    def get(port):
        """Get input port value
    :param port: input port number
    :type port: int in range (0, 112)
        """
        # r_print(in_ports)
        return in_ports[0][port]

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

    #######################################################

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
        with closing(pg.connect(user='postgres', password='panda', host='localhost', database='postgres')) as conn:
            with conn.cursor() as cursor:
                cursor.execute(query)
                conn.commit()
                if result:
                    return cursor.fetchall()
    
    @staticmethod
    def getsizes(shares, amount):
        """Transform wallet shares to category sizes optimally
    catsizes = [cs1, cs2, cs3, cs4] - blocks numbers of each color category
    :param shares: list of wallet shares
    :type shares: list in range (4)
    :param amount: total amount of blocks
    :type amount: int
    :return: list of category sizes
    :rtype: list in range (4)
        """

        catsizes = [w * amount for w in shares]

        for catsize in catsizes:
            # if any block catsize is non-integer...
            if catsize - int(catsize) > 0:
                # ==============================================================
                # Round all shares optimally (0.5, +1, -1)
                trig = True
                for k, cs in enumerate(catsizes):
                    if cs - int(cs) == 0.5:
                        if trig:
                            catsizes[k] = int(cs + 0.5)
                            trig = False
                        else:
                            catsizes[k] = int(cs - 0.5)
                            trig = True
                    else:
                        catsizes[k] = round(cs)
                # ==============================================================
                if amount - sum(catsizes) == 1:
                    maxcat = max([cs - int(cs) for cs in catsizes])
                    for k, cs in enumerate(catsizes):
                        if cs - int(cs) == maxcat:
                            catsizes[k] += 1
                            break
                elif sum(catsizes) - amount == 1:
                    mincat = min([cs - int(cs) for cs in catsizes])
                    for k, cs in reversed(list(enumerate(catsizes))):
                        if cs - int(cs) == mincat:
                            catsizes[k] -= 1
                            break
                # ==============================================================
                return catsizes
        else:
            return [int(cs) for cs in catsizes]

    @staticmethod
    def getqueue():
        """Transform category sizes to block queue optimally
    catsizes = [cs1, cs2, cs3, cs4] - blocks numbers of each color category
        """

        global shares
        if not shares:
            return 0

        # Defining catsizes matching the MSE-limit
        amount = 1  # starting amount
        lim = 0.03  # MSE-limit
        while True:
            error = 0
            catsizes = self.getsizes(shares, amount)
            for cs, w in zip(catsizes, shares):
                error += (cs / amount - w) ** 2
            error = m.sqrt(error / 4)
            if error > lim:
                amount += 1
            else:
                break

        # ======================================================================
        # Evenly distributing algorithm of block queue using dimensional method
        # (catsizes = (cs1; cs2; cs3; cs4) - 4D-vector)

        fullvec = sum([cs * cs for cs in catsizes])
        passedvec = 0

        point = [0] * 4
        delta = [0] * 4

        queue = []

        for _ in range(sum(catsizes)):
            # Defining the minimal delta for each point (???)
            for coord in range(4):
                delta[coord] = (2 * point[coord] + 1) * fullvec - (2 * passedvec + catsizes[coord]) * catsizes[coord]

            bestcoord = delta.index(min(delta))
            passedvec += catsizes[bestcoord]
            point[bestcoord] += 1

            queue.append(bestcoord)
        # ======================================================================
        return queue

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
        in_ports.append(msg.data)

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
        shares.append(msg.data)

    spectator = rospy.Subscriber("spectator",
                                 Float64MultiArray,
                                 __spectator_update,
                                 queue_size=500)

    ######################################
    #           ROS Publisher            #
    ######################################

    pub = rospy.Publisher("modbus_wrapper/output",
                          Int32MultiArray,
                          queue_size=500)

    output = Int32MultiArray()
