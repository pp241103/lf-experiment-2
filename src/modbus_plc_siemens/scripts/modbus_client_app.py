#!/usr/bin/env python
import gnureadline
import sys

from contextlib import suppress
from modbus_plc_siemens.algorithms import MainApi
from modbus_plc_siemens.client_init import ModbusClient
from modbus_plc_siemens.base_api import rospy, in_ports

##################################################################################

if __name__ == "__main__":

    ######################################
    #           Initialisation           #
    ######################################

    rospy.init_node("modbus_client_app")

    modbus_host = "192.168.88.199"
    modbus_port = 502

    modclient = None
    R = MainApi()

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        R.print("Modbus client session started")

    except Exception as e:
        R.error("Modbus client session didn't start")
        R.error(f"REASON: {e}")
        exit(1)
    
    # filling in_ports by pressing any button
    R.print("(Press any button on the factory to proceed)")
    while not in_ports:
        R.sleep(0.001)

    ######################################
    #            Application             #
    ######################################

    R.print("Available commands: "
            "run, stop, exit / "
            "show(.), fill, clear, add*(.) / "
            "act*(.), set(.) / "
            "color, shares")

    command = None
    methods = {'run': R.run_factory,
               'stop': R.stop_factory,
               'fill': R.fill_warehouse,
               # 'info': describe_methods,
               'clear': R.clear_warehouse,
               'color': R.get_color,
               'shares': R.check_shares}

    while command != "exit":
        command = input("Command: ")

        try:
            if command in ('run', 'stop', 'fill', 'clear', 'color', 'shares'):
                methods[command]()
            elif command.startswith(('act', 'add', 'set', 'show')):
                exec(f'R.{command}')
            elif command == "exit":
                
                # application closing
                sys.tracebacklimit = 0
                with suppress(Exception):
                    R.print("Modbus client session is shutting down")
                    rospy.signal_shutdown("server shutting down")
                    # modclient.stopListening()

            else:
                print('- This command is not available!')
                # exec(command)

        except Exception as e:
            R.error('Error while executing the command')
            R.error(f"REASON: {e}")
