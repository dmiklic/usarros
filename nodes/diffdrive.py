#!/usr/bin/env python

# ROS core imports
import roslib; roslib.load_manifest('usarros')
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

# Custom ROS imports
from usar_config_parser import usar_config_parser

# Imports from this package
from usar_client import usar_client

# Third-party library imports
from numpy import array, sqrt, unwrap, pi

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
                              'GroundTruth': self.handle_truth_msg,
                              'Odometry': self.handle_odometry_msg} 
        self.geo_msg_handlers = {'GroundVehicle': self.handle_veh_geo,
                              'RangeScanner': self.handle_scanner_geo}        

        # Initialize robot parameters
        self.name = name
        self.r = None    # wheel radius
        self.l = None    # half axle length
        
        # Initialize ROS
        rospy.init_node('usardiff')
        
        # Initialize parameters from ROS parameter server
        tf_prefix = ''        

        # Initialize robot positions/velocities
        self.ground_truth = Odometry()
        self.ground_truth.header.stamp = rospy.Time.now()
        self.ground_truth.header.frame_id = tf_prefix + 'odom'
        self.ground_truth.child_frame_id = tf_prefix + 'base_link'
        self.ground_truth.pose.pose.position.x = vehpar['x']
        self.ground_truth.pose.pose.position.y = vehpar['y']
        self.ground_truth.pose.pose.position.z = vehpar['z']        
        rot_quat = tf.transformations.quaternion_from_euler(vehpar['yaw'],
                                                       vehpar['pitch'],
                                                       vehpar['roll'])
        self.ground_truth.pose.pose.orientation.x = rot_quat[0]
        self.ground_truth.pose.pose.orientation.y = rot_quat[1]
        self.ground_truth.pose.pose.orientation.z = rot_quat[2]
        self.ground_truth.pose.pose.orientation.w = rot_quat[3]
       
        self.odom = Odometry()
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = tf_prefix + 'odom'
        self.odom.child_frame_id = tf_prefix + 'base_link'

        # Store the tf_prefix
        self.tf_prefix = tf_prefix

        # Create publishers and subscribers
        self.scan_pub = rospy.Publisher('scan', LaserScan)
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.truth_pub = rospy.Publisher('base_pose_ground_truth', Odometry)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # Connect to USARSim       
        self.client = usar_client(usarpar['hostname'], usarpar['port'], 
                        self.parse_msg, self.create_init_msg(vehpar['type'], name))


#------------------------------------------------------------------------------

    def create_init_msg(self, vehtype, name):
        """ Create USARSim command to spawn the robot and get configuration information. """
               
        self.tf_listener.waitForTransform('usarsim', 'odom', rospy.Time(), rospy.Duration(60))
        
        # Transform ROS (odom) pose to USARSim
        pose_usar = self.tf_odom_msg_pose('usarsim', self.ground_truth)
        q = pose_usar.pose.orientation        
        rot_usar = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])               
        
        # Spawn spawn the robot     
        msg = ('INIT {{ClassName USARBot.{0}}} '.format(vehtype)
            + '{{Name {0}}} '.format(name) 
            + '{{Location {0},{1},{2}}} '.format(pose_usar.pose.position.x, 
                                                pose_usar.pose.position.y, 
                                                pose_usar.pose.position.z)
            + '{{Rotation {0},{1},{2}}}\r\n'.format(rot_usar[2], rot_usar[1], rot_usar[0]))

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
        if self.r and self.l:
            time_now = rospy.Time.now()
            # Broadcast the odom transform first
            odom_pos = self.odom.pose.pose.position
            odom_rot = self.odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform((odom_pos.x, odom_pos.y, odom_pos.z),
                                          (odom_rot.x, odom_rot.y, odom_rot.z, odom_rot.w),
                                          time_now,
                                          'base_link',
                                          'odom')            
            for (name, par) in args.items():
                par = par.split(',')            
                if name == 'Vel':
                    w_left = float(par[0])
                    w_right = float(par[1])
                    lin_vel = (w_left + w_right) * self.r / 2.0
                    rot_vel = self.r * (w_right - w_left) / self.l
            self.odom.twist.twist.linear.x = lin_vel        
            self.odom.twist.twist.angular.z = rot_vel
            self.odom.header.stamp = time_now
            # Publish the odom message
            self.odom_pub.publish(self.odom)
        else:
            rospy.logwarn("Can't compute velocity, robot parameters unknown!")    
#------------------------------------------------------------------------------

    def handle_scanner_msg(self, args):
        for (name, par) in args.items():
            pass            
            #print(name)

#------------------------------------------------------------------------------

    def handle_odometry_msg(self, args):
        for (name, par) in args.items():
            if name == 'Pose':
                # Unpack data sent by usarsim                
                time_now = rospy.Time.now()                    
                xyyaw = [float(s) for s in par.split(',')]
                self.odom.pose.pose.position.x = xyyaw[0]
                self.odom.pose.pose.position.y = xyyaw[1]
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, xyyaw[2])
                self.odom.pose.pose.orientation.x = q[0]
                self.odom.pose.pose.orientation.y = q[1]
                self.odom.pose.pose.orientation.z = q[2]
                self.odom.pose.pose.orientation.w = q[3]
                self.odom.header.frame_id = 'usarsim'
                # Transform to ROS (odom) cordinates
                odom_pose = self.tf_odom_msg_pose('odom', self.odom)                
                self.odom.header = odom_pose.header
                self.odom.header.stamp = time_now
                self.odom.pose.pose = odom_pose.pose
                
                # We will not publish the message
                # We will let handle_tacho_msg publish the whole odom message

#------------------------------------------------------------------------------

    def handle_truth_msg(self, args):
        """ Read ground truth info and publish it """                      
        time_now = rospy.Time.now()        
        dt = (time_now - self.ground_truth.header.stamp).to_sec()
        self.ground_truth.header.stamp = time_now
        
        # Store old positions for speed calculations        
        x_old = self.ground_truth.pose.pose.position.x
        y_old = self.ground_truth.pose.pose.position.y
        q = self.ground_truth.pose.pose.orientation
        yaw_old = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]        

        for (name, par) in args.items():            
            if name == 'Location':
                self.set_pos(self.ground_truth.pose.pose.position, par)                
            elif name == 'Orientation':
                self.set_rot(self.ground_truth.pose.pose.orientation, par)
        
        # Transform from USARSim to ROS (odom) coordinates                
        self.ground_truth.header.frame_id = 'usarsim'        
        truth_pose = self.tf_odom_msg_pose('odom', self.ground_truth)                
        self.ground_truth.header = truth_pose.header
        self.ground_truth.header.stamp = time_now
        self.ground_truth.pose.pose = truth_pose.pose

        # The USARSim GroundTruth sensor is not publishing velocities
        # so we'll compute them by hand
        pos_new = self.ground_truth.pose.pose.position
        lin_vel = sqrt((pos_new.x-x_old)**2+(pos_new.y-y_old)**2) / dt    
        self.ground_truth.twist.twist.linear.x = lin_vel
        q = self.ground_truth.pose.pose.orientation
        yaw_new = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]        
        self.ground_truth.twist.twist.angular.z = myunwrap(yaw_new - yaw_old) / dt        
        
        # Publish the ground truth message.        
        self.truth_pub.publish(self.ground_truth)

#------------------------------------------------------------------------------

    def tf_odom_msg_pose(self, target_frame, odom_msg, ts=rospy.Time()):
        """ Utility function for transforming the pose part of an Odometry message. 
            It assumes static frames (twist is not transformed!)
        """
        pose = PoseStamped()        
        pose.header.frame_id = odom_msg.header.frame_id        
        pose.pose.position = odom_msg.pose.pose.position
        pose.pose.orientation = odom_msg.pose.pose.orientation
        pose.header.stamp = ts        
        
        return self.tf_listener.transformPose(target_frame, pose)        

#------------------------------------------------------------------------------

    def set_pos(self, position_msg, pos):
        """ Sets position data in a position message """
        pos = [float(s) for s in pos.split(',')]        
        position_msg.x = pos[0]
        position_msg.y = pos[1]    # Account for different coordinate system in USARSim
        position_msg.z = pos[2]

#------------------------------------------------------------------------------
    
    def set_rot(self, orientation_msg, rot):
        """ Sets orientation data in a quaternion """
        rot = [float(s) for s in rot.split(',')]   
        # quaternion_from_euler expects roll,pitch,yaw??? BUG?     
        rot_quat = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])        
        orientation_msg.x = rot_quat[0]
        orientation_msg.y = rot_quat[1]
        orientation_msg.z = rot_quat[2]    # Account for different coordinate system in USARSim
        orientation_msg.w = rot_quat[3]

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

    def run(self):
        """ Keep communicating with usarsim """
        asyncore.loop()

#==============================================================================

def myunwrap(angle):
    arr = array([0.0, angle])
    arr = unwrap(arr)
    return arr[1]

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

