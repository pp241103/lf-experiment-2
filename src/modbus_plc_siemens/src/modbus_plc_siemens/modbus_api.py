from pymodbus.client.sync import ModbusTcpClient
from threading import Lock

def print_error(error):
    print(f"\033[31m{error}\033[0m")


# -----------------------------------------------------
class ModbusApi:
    def __init__(self, host="192.168.0.1", port="500"):
        self.mutex = Lock()

        # reading registers
        self.readingStart = 0
        self.readingCount = 112

        # writing registers
        self.writingStart = 112
        self.writingCount = 106

        try:
            self.client = ModbusTcpClient(host, port)
        except Exception as e:
            print_error(f"\n- Modbus error: {e}")
            
    # -------------------------------------------------
    def start_socket(self):
        try:
            self.client.connect()
            if not self.client.is_socket_open():
                return -1
            return 0
        except Exception as e:
            print_error(f"\n- Modbus error: {e}")
            return -1
            
    # -------------------------------------------------
    def stop_socket(self):
        try:
            self.client.close()
        except Exception as e:
            print_error(f"\n- Modbus error: {e}")
            
    # -------------------------------------------------
    def set_register(self, index, value):
        if not (0 <= index < self.writingCount) or (value not in (0, 1)):
            print_error(f"- Error: Incorrect index or value")
            return -2
        index = self.writingStart + index
        with self.mutex:
            try:
                if not self.client.is_socket_open():
                    return -1
                self.client.write_register(index, value)
                return 0
            except Exception as e:
                print_error(f"\n- Modbus error: {e}")
                return -1
                
    # -------------------------------------------------
    def null_registers(self):
        values = [0] * self.writingCount

        with self.mutex:
            try:
                self.client.write_registers(self.writingStart, values)
                return 0
            except Exception as e:
                print_error(f"\n- Modbus error: {e}")
                return -1
            
    # -------------------------------------------------
    def get_register(self, index):
        if not (0 <= index < self.readingCount):
            print_error(f"- Error: Incorrect index")
            return -2
        index = self.readingStart + index
        with self.mutex:
            try:
                if not self.client.is_socket_open():
                    return -1
                request = self.client.read_holding_registers(index, 1)
                if request.isError():
                    print_error(f"\n- Modbus error: {e}")
                    return -1
                return request.registers[0]
            except Exception as e:
                print_error(f"\n- Modbus error: {e}")
                return -1
