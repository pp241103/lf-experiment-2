#!/usr/bin/env python
import gnureadline
import sys

from contextlib import suppress
from modbus_plc_siemens.base_api import rospy
from modbus_plc_siemens.algorithms import MainApi
from modbus_plc_siemens.client_init import ModbusClient

##################################################################################

if __name__ == "__main__":

    ######################################
    #           Initialisation           #
    ######################################

    rospy.init_node("modbus_client_app")

    modbus_host = "192.168.0.199"
    modbus_port = 502

    modclient = None
    R = MainApi()

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        R.print("Modbus client session started")

    except Exception as e:
        R.error("Modbus client session didn't start")
        R.error(f"[REASON] {e}")
        exit(1)

    # in_ports = modclient.readRegisters(0, 112)
    # r_print(out_ports)

    ######################################
    #            Application             #
    ######################################

    R.print("Available commands: "
            "run, stop, exit / "
            "show, clear, add*(.) / "
            "act*(.), set(.), color / ")

    command = None
    methods = {'run': R.run_factory,
               'stop': R.stop_factory,
               'show': R.show_warehouse,
               'clear': R.clear_warehouse,
               'color': R.get_color}

    while command != "exit":
        command = input("Command: ")

        try:
            if command in ('run', 'stop', 'show', 'clear', 'color'):
                methods[command]()
            elif command.startswith(("add", "act", "set")):
                exec('R.' + command)
            elif command == "exit":
                
                # application closing
                sys.tracebacklimit = 0
                with suppress(Exception):
                    R.print("Modbus client session is shutting down")
                    rospy.signal_shutdown("server shutting down")
                    modclient.stopListening()

            else:
                print('- This command is not available!')
                # exec(command)

        except Exception as e:
            R.error('while executing the command')
            R.error(f"[REASON] {e}")
