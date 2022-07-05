from modbus.modbus_wrapper import ModbusWrapperClient

# ===================================================

class ModbusClient(ModbusWrapperClient):
    def __init__(self, host, port=502, rate=50):
        """Initialize the modbus client specially
    :param host: IP address of the modbus server
    :type host: str
    :param port: port of the modbus server, defaults to 502
    :type host: int
    :param rate: registers reading frequency per second, defaults to 50
    :type rate: int
        """

        ModbusWrapperClient.__init__(self, host, port, rate)
        
        self.setWritingRegisters(112, 106)
        self.setReadingRegisters(0, 112)
        self.startListening()

# ===================================================













































































