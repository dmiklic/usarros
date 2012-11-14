#!/usr/bin/env python

# ROS core imports
import roslib; roslib.load_manifest('usarros')
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Custom ROS imports
from usar_config_parser import usar_config_parser

# Imports from this package
from usar_client import usar_client

# Standard library imports
import sys 
import asyncore

#==============================================================================

class diffdrive:
    """ A differential drive robot, simulated in USARSim"""

    def __init__(self, config_file, name):
        """
            Parameters:
            -----------
            config_file : Name of a .cfg file, containing configuration data.
            name       : The name of the robot.
        """
        
        cfg = usar_config_parser()
        cfg.readfp(open(config_file))
        vehpar = cfg.parsed_items('vehicles')[name]
        usarpar = cfg.section_items('usarsim')
        
        # Assign message handlers to USARSim message types        
        self.sen_msg_handlers = {'Tachometer': self.handle_tacho_msg,
                              'RangeScanner': self.handle_scanner_msg,
                              'GroundTruth': self.handle_truth_msg} 
        self.geo_msg_handlers = {'GroundVehicle': self.handle_veh_geo,
                              'RangeScanner': self.handle_scanner_geo}        

        # Initialize robot parameters
        self.name = name
        self.r = None    # wheel radius
        self.l = None    # half axle length
        self.pos_gt_0 = [vehpar['x'], vehpar['y'], vehpar['z']]   # ground truth, initial pose
        self.pos_gt = self.pos_gt_0[:]        
        self.pos_odom = [0.0, 0.0, 0.0] # odometry
        self.rot_gt_0 = [vehpar['roll'], vehpar['pitch'], vehpar['yaw']]   # initial orientation
        self.rot_gt = self.rot_gt_0[:]
        self.rot_odom = [0.0, 0.0, 0.0]
        self.linvel_gt = [0.0,0.0,0.0]
        self.linvel_odom = [0.0,0.0,0.0]
        self.rotvel_gt = [0.0,0.0,0.0]
        self.rotvel_odom = [0.0,0.0,0.0]
        
        # Attached device parameters
        # TODO: Fix this (and other stuff) to support multiple devices
        # Devices should probably become objects        
        self.laser_pos = [0.0,0.0,0.0]        
        self.laser_rot = [0.0,0.0,0.0]

        # Connect to USARSim       
        self.client = usar_client(usarpar['hostname'], usarpar['port'], 
                        self.parse_msg, self.create_init_msg(vehpar['type'], name))

        # Initialize ROS        
        self.truth_pub = rospy.Publisher('base_pose_ground_truth', Odometry)
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.init_node('usardiff')

        # Initialize parameters from ROS parameter server
        self.tf_prefix = ''

#------------------------------------------------------------------------------

    def create_init_msg(self, vehtype, name):
        """ Create USARSim command to spawn the robot and get configuration information. """
        # TODO: Transform ROS map coordinates to USARSim coordinates        
        
        # Spawn spawn the robot     
        msg = ('INIT {{ClassName USARBot.{0}}} '.format(vehtype)
            + '{{Name {0}}} '.format(name) 
            + '{{Location {0},{1},{2}}} '.format(self.pos_gt[0], self.pos_gt[1], self.pos_gt[2])
            + '{{Rotation {0},{1},{2}}}\r\n'.format(self.rot_gt[0], self.rot_gt[1], self.rot_gt[2]))

        # Query vehicle configuration
        msg += 'GETGEO {Type Robot}\r\n'        
        # Add Laser configuration query message
        msg += 'GETGEO {Type RangeScanner}\r\n'
        return msg

#------------------------------------------------------------------------------

    def parse_msg(self, msg):
        """ Handles incoming USARSim messages """        
        #print(msg)        
        args = dict()        
        parts = msg.split('{')        
        msg_name = parts[0].strip()
        for part in parts[1:]:
            part = part.strip()
            part = part[:-1]    # Remove trailing }
            arg = part.split(' ')
            name = arg[0]                        
            if len(arg) > 1:
                value = arg[1]
            else:
                value = ''            
            args[name] = value
        if msg_name == 'SEN':
            self.sen_msg_handlers[args['Type']](args)
        elif msg_name == 'GEO':
            self.geo_msg_handlers[args['Type']](args)
        elif msg_name == 'STA':
            self.handle_status_msg(args)

#------------------------------------------------------------------------------            

    def handle_veh_geo(self, args):        
        """ Sets vehicle physical properties 
            NOTE: The data provided by USARSim is currently not reliable!
        """        
        for (name, par) in args.items():
            if name == 'WheelRadius':
                self.r = 0.098  # Hardcoded for Kiva (?)
            elif name == 'WheelBase':
                self.l = 2*0.324 # Hardcoded fo Kiva (?)

#------------------------------------------------------------------------------
                
    def handle_scanner_geo(self, args):
        for (name,par) in args.items():
            print(name, par)

#------------------------------------------------------------------------------

    def handle_status_msg(self, args):
        """ Handling status messages
            We're not really interested in those.
            We'll just use this to send commands to USARSim
        """
        self.client.send_data()

#------------------------------------------------------------------------------

    def handle_tacho_msg(self, args):
        for (name, par) in args.items():
            pass
            #print(name)

#------------------------------------------------------------------------------

    def handle_scanner_msg(self, args):
        for (name, par) in args.items():
            pass            
            #print(name)

#------------------------------------------------------------------------------

    def handle_truth_msg(self, args):
        """ Process ground truth info """              
        for (name, par) in args.items():
            par = par.split(',')            
            if name == 'Location':
                self.pos_gt = [float(s) for s in par]
            elif name == 'Orientation':
                self.rot_gt = [float(s) for s in par]
            # TODO: Add velocity publishing to USARSim GroundTruth sensor
        # TODO: Publish ground truth transform here!        
        self.publish_odom(self.pos_gt, self.rot_gt, 
                        self.linvel_gt, self.rotvel_gt,
                        self.truth_pub)

#------------------------------------------------------------------------------

    def cmd_vel_callback(self, data):        
        if (self.r and self.l):            
            v = data.linear.x
            w = data.angular.z
            w_left = v/self.r - self.l*w/(2*self.r)
            w_right = v/self.r + self.l*w/(2*self.r)

            w_left = w_left/10.0
            w_right = w_right/10.0

            usar_msg = ('DRIVE {{Name {0}}} '.format(self.name)
                      + '{{Left {0}}} '.format(w_left)
                      + '{{Right {0}}}\r\n'.format(w_right))
            # This callback runs in its own thread
            # Our asynchat-based client is not thread-safe
            # Therefore, we will just queue the message here
            # And we'll send it later            
            self.client.queue_msg(usar_msg)
        else:
            rospy.logwarn("Can't issue DRIVE command, robot parameters unknown!")

#------------------------------------------------------------------------------

    def publish_odom(self, pos, rot, linvel, rotvel, pub):
        """ Assemble an odom message from position, orientation 
            and velocity data, publish it to the provided channel
        """

        # Publish the odometry message.        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()         
        odom.header.frame_id = self.tf_prefix + '/ground_truth'
        odom.child_frame_id = self.tf_prefix + '/base_link'        
        
        # Position        
        odom.pose.pose.position.x = self.pos_gt[0]
        odom.pose.pose.position.y = self.pos_gt[1]
        odom.pose.pose.position.z = self.pos_gt[2]       
        
        # Orientation
        rot_quat = tf.transformations.quaternion_from_euler(self.rot_gt[0],
                                                       self.rot_gt[1],
                                                       self.rot_gt[2])        
        odom.pose.pose.orientation.x = rot_quat[0]
        odom.pose.pose.orientation.y = rot_quat[1]
        odom.pose.pose.orientation.z = rot_quat[2]
        odom.pose.pose.orientation.w = rot_quat[3]  
        
        # Velocity
        odom.twist.twist.linear.x = self.linvel_gt[0]
        odom.twist.twist.linear.y = self.linvel_gt[1]
        odom.twist.twist.linear.z = self.linvel_gt[2]
        odom.twist.twist.angular.x = self.rotvel_gt[0]
        odom.twist.twist.angular.y = self.rotvel_gt[1]
        odom.twist.twist.angular.z = self.rotvel_gt[2]
       
        pub.publish(odom)

#------------------------------------------------------------------------------

    def run(self):
        """ Keep communicating with usarsim """
        asyncore.loop()

#==============================================================================

def parse_arguments(argv):
    """ Parse the command line arguments.
        Returns a dictionary where argument names are keys,
        and argumens are values.
    """     
    usage = ('Usage: rosrun usarros diffdrive.py ' + 
            'config=<configfile.cfg> <robot=robotname>')    
    # Check input parameters
    assert len(argv) > 1, usage   
    args = {}
    for s in argv[1:]:
        parts = s.split('=')
        args[parts[0]] = parts[1]
    
    return args

#==============================================================================

if __name__ == '__main__':
    
    args = parse_arguments(sys.argv)    
    vehicle = diffdrive(args['config'], args['robot'])
    vehicle.run()

