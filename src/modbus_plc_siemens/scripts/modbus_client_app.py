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
    M = MainApi()

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        M.print("Modbus client session started")

    except Exception as e:
        M.error("Modbus client session didn't start")
        M.error(f"REASON: {e}")
        exit(1)
    
    # filling in_ports by pressing any button
    M.print("(Press any button on the factory to proceed)")
    while not in_ports:
        M.sleep(0.001)
    
    # ----------------------------------------------------------------------------
    
    ######################################
    #            Application             #
    ######################################

    M.print("Main commands: run, stop, exit\n"+
            ' '*28+"Checking commands: color, shares\n"+
            ' '*28+"Database commands: add (.), show (.), fill, clear\n"+
            ' '*28+"Service commands: act0 (.), act1 (.), act2 (.), act3 (.), set (.), time (.)")
    M.print("Command pattern: \'COMMAND\' or \'COMMAND ARGUMENTS\'")

    command = None
    methods = {'color': 'M.get_color',
               'shares': 'M.check_shares',
               # 'info': describe_methods,
               
               'run': 'M.run_factory',
               'stop': 'M.stop_factory',
               
               'add': 'M.add_blocks',
               'show': 'M.show_warehouse',
               'fill': 'M.fill_warehouse',
               'clear': 'M.clear_warehouse',
               
               'act1': 'M.act1', 'act1': 'M.act1', 'act1': 'M.act1', 'act1': 'M.act1',
               'set': 'M.set', 'time': 'M.set_time'}

    while command != "exit":
        command = input("Command: ").split(maxsplit=1)
        command, args = (command[0], '') if len(command) == 1 else command

        try:
            if command in ('run', 'stop',
                           'color', 'shares',
                           'show', 'fill', 'clear', 'add',
                           'act0', 'act1', 'act2', 'act3', 'set'):
                exec(f'{methods[command]}({args})')
            elif command == "exit":
                
                # application closing
                sys.tracebacklimit = 0
                with suppress(Exception):
                    M.print("Modbus client session is shutting down")
                    rospy.signal_shutdown("server shutting down")
                    # modclient.stopListening()
            else:
                print('- This command is not available!')
                # exec(command)

        except Exception as e:
            M.error('Error while executing the command')
            M.error(f"REASON: {e}")
        
##################################################################################





