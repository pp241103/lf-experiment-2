import psycopg2 as pg
import time as t
import rospy

from modbus_plc_siemens.modbus_api import ModbusApi
from modbus_plc_siemens.post_threading import Post
from std_msgs.msg import Float64MultiArray
from contextlib import closing


######################################################################################

def print_message(message):
    print(f"\033[36m{message}\033[0m")


def print_error(error):
    print(f"\033[31m{error}\033[0m")


class BaseApi:

    def __init__(self, host, port):
        self.shares = []

        rospy.init_node("LF_project_app")
        rospy.Subscriber("spectator",
                         Float64MultiArray,
                         self.spectator_update,
                         queue_size=500)

        # there will be publisher + subscriber
        # for web-application as another process

        self.post = Post(self)

        self.client = ModbusApi(host, port)
        if self.client.start_socket() == -1:
            self.exit_program(1) # failed connection

        print_message("- Client was succesfully initialized!")
        print_message(f"  ({host}:{port})")

    ##################################################################################

    @staticmethod
    def sleep(time: float):
        """Sleep timer
    :param time: seconds to sleep
    :type time: float
        """
        t.sleep(time)

    @staticmethod
    def print_help():
        print_message("\n- Main commands: \n" +
                " 1. run    | run the factory \n" +
                " 2. stop   | stop the factory \n" +
                " 2. exit   | exit the program \n" +
                " 3. break  | break the process")
        print_message("\n- Checking commands: \n" +
                " 1. get    | get register value \n" +
                " 2. clr    | check actual sensor' color \n" +
                " 3. shrs   | check actual wallets' shares")
        print_message("\n- Database commands: \n" +
                " 1. add    | fill scecific cells in departure' table \n" +
                " 2. show   | show all departure/arrival' table \n" +
                " 3. fill   | fill all departure' table \n" +
                " 4. clear  | clear all arrival' table")
        print_message("\n- Service commands: \n" +
                " 1. time   | set time of block' handling \n" +
                " 2. acts   | move loaders in specific way \n" +
                " 3. set    | set register in specific way")
        print_message("\nCommand pattern: \'COMMAND [ARG1 ARG2 ...]\'\n")

    def exit_program(self, code=0):
        self.post.killall()
        
        self.client.null_registers()
        self.client.stop_socket()

        rospy.signal_shutdown("\nRospy is shutting down...")
        print_message("\nClient is shutting down...")
        exit(code) # hard exit

    ##################################################################################

    def get(self, port: int):
        """Get input port value
    :param port: input port number
    :type port: int in range (0, 112)
        """
        value = self.client.get_register(port)
        if value == -1: # lost connection
            self.exit_program(1)
        return value

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

        ret = self.client.set_register(port, value)
        if ret == -1: # lost connection
            self.exit_program(1)
        elif ret == -2:
            return

        if value == 0 or (not sensor and not time):
            return
        if sensor:
            value = self.get(sensor)
            while self.get(sensor) == value:
                self.sleep(0.001)
        if time:
            self.sleep(time)

        ret = self.client.set_register(port, 0)
        if ret == -1: # lost connection
            self.exit_program(1)

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

    def check_shares(self):
        """Print wallet shares"""
        print_message(f'- Wallet shares: {self.shares}')

    def get_color(self):
        """Get current color from analog sensor"""
        print_message(f'- Current color: {self.get(0)}')

    def get_register(self, port):
        print_message(f'- Register {port}: {self.get(port)}')

    ##################################################################################

    def spectator_update(self, msg):
        """Service callback-function:
    Read wallet shares and write them to list
    Make wallet shares available in app
        """
        del self.shares[0: len(self.shares)]
        self.shares.extend(msg.data)

# ------------------------------------------------------------------------------------
