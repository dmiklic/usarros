import asynchat
import socket
import threading
import rospy

class usar_client(asynchat.async_chat):
    """Sends messages to USARSim and receives responses.
    """
    
    def __init__(self, host, port, reader, init_msg):
        """
            Parameters:
            -----------
            host : Name of the USARSim host
            port : USARSim port number
            reader : Function for processing incoming messages
            spawn_msg : UARSim command string for spawning a robot
        """        
        asynchat.async_chat.__init__(self)
        #self.sending = threading.Lock()        
        self.inbuffer = ''
        self.outbuffer = ''
        self.reader = reader
        self.init_msg = init_msg
        self.set_terminator('\r\n')
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
        return
        
    def queue_msg(self, data):    
        """ Queue message for sending """
        #self.sending.acquire()                
        self.outbuffer += data
        #print('Outbuffer: %s' % self.outbuffer)
        #self.sending.release()
    
    def send_data(self):
        """ Sends data from the outbuffer 
            Calls asynchat.push() which is not threadsafe, so make sure
            you call this from only one thread!
        """
        self.push(self.outbuffer)
        self.outbuffer = ''
   
    def handle_connect(self):
        """ Upon connection, we spawn the vehicle.
        """
        self.queue_msg(self.init_msg)
        self.send_data()

    def collect_incoming_data(self, data):
        self.inbuffer += data

    def found_terminator(self):
        #print(self.inbuffer)        
        self.reader(self.inbuffer)
        self.inbuffer = ''


