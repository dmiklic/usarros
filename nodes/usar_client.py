import asynchat
import socket

class usar_client(asynchat.async_chat):
    """Sends messages to USARSim and receives responses.
    """
    
    def __init__(self, host, port, reader, spawn_msg):
        """
            Parameters:
            -----------
            host : Name of the USARSim host
            port : USARSim port number
            reader : Function for processing incoming messages
            spawn_msg : UARSim command string for spawning a robot
        """        
        asynchat.async_chat.__init__(self)
        self.inbuffer = ''
        self.reader = reader
        self.spawn_msg = spawn_msg
        self.set_terminator('\r\n')
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
        return
        
    def handle_connect(self):
        """ Upon connection, we spawn the vehicle.
        """
        self.push(self.spawn_msg)

    def collect_incoming_data(self, data):
        self.inbuffer += data

    def found_terminator(self):
        self.reader(self.inbuffer)
        self.inbuffer = ""


