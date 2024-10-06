import serial

class SerialSingleton:
    _instance = None

    def __new__(cls, port, baudrate, timeout):
        if cls._instance is None:
            cls._instance = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,  
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout)
        return cls._instance
