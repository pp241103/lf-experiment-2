#!/usr/bin/env python

from modbus_plc_siemens.algorithms import MainApi
from modbus_plc_siemens.base_api import print_error
from modbus_plc_siemens.labels import *

import gnureadline

#  Peter
import asyncio
import websockets
from threading import Thread

##################################################################################

async def in_command():
    return input("Command: ").split()

async def main(host, port):
    lf = MainApi(host, port)
    lf.print_help()

    methods = {
        'run': 'lf.run_factory',
        'stop': 'lf.stop_factory',
        'break': 'lf.break_factory',

        'help': 'lf.print_help',
        'exit': 'lf.exit_program',

        'clr': 'lf.get_color',
        'shrs': 'lf.check_shares',

        'set': 'lf.set',
        'time': 'lf.set_time',
        'get': 'lf.get_register',

        'add': 'lf.add_blocks',
        'show': 'lf.show_warehouse',
        'fill': 'lf.fill_warehouse',
        'clear': 'lf.clear_warehouse',

        'act1': 'lf.act1',
        'act2': 'lf.act2',
        'act3': 'lf.act3',
        'act4': 'lf.act4'
    }


    # ----------------------------------------------------------------------------

    while True:
        command = await in_command()
        if not command:
            continue
        args = "" if (len(command) == 1) else ",".join(command[1:])

        if command[0] in (
                'help', 'exit',
                'clr', 'shrs',
                'set', 'time', 'get',
                'run', 'stop', 'break',
                'add', 'show', 'fill', 'clear',
                'act0', 'act1', 'act2', 'act3'
        ):
            try:
                exec(f'{methods[command[0]]}({args})')
            except Exception as e:
                print_error(f"- Execution error: {e}")
        else:
            print_error("- Warning: this command is not available!")


##################################################################################

#  Peter
async def handler(websocket):
    async for message in websocket:
        print(message)

async def run_ws_server():
    async with websockets.serve(handler, "", 8001):
        await asyncio.Future()  # run forever

def main_ws():
    asyncio.run(run_ws_server())

##################################################################################

if __name__ == "__main__":
    Thread(target = main_ws).start()
    asyncio.run(main("192.168.88.199", 502))
