#!/usr/bin/env python

# Standard library imports
import sys 
import asyncore

# Third-party library imports
from numpy import array, sqrt, unwrap, pi

# ROS core imports
import roslib; roslib.load_manifest('usarros')
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Custom ROS imports
from usar_config_parser import usar_config_parser

# Imports from this package
from usar_client import usar_client

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
        tf_prefix = rospy.get_param('~tf_prefix', '')
        if tf_prefix:
            tf_prefix = '/' + tf_prefix + '/'
        else:
            tf_prefix = '/'        
        self.usarsim_frame = tf_prefix + 'usarsim'
        self.odom_frame = tf_prefix + 'odom'
        self.base_link_frame = tf_prefix + 'base_link'
        self.base_laser_frame = tf_prefix + 'base_laser'

        # Initialize robot positions/velocities
        self.ground_truth = Odometry()
        self.ground_truth.header.stamp = rospy.Time.now()
        self.ground_truth.header.frame_id = self.odom_frame
        self.ground_truth.child_frame_id = self.base_link_frame
        self.ground_truth.pose.pose.position = Point(vehpar['x'],
                                                 vehpar['y'],
                                                 vehpar['z'])        
        rot_quat = quaternion_from_euler(vehpar['roll'],
                                      vehpar['pitch'],
                                      vehpar['yaw'])
        self.ground_truth.pose.pose.orientation = Quaternion(*rot_quat)
       
        self.odom = Odometry()
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = self.odom_frame
        self.odom.child_frame_id = self.base_link_frame

        # Initialize sensors
        self.laser_tf = TransformStamped()
        self.laser_tf.header.frame_id = self.base_link_frame
        self.laser_tf.child_frame_id = self.base_laser_frame

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

        # Stop the client when we receive a shudown request
        rospy.on_shutdown(self.client.close_when_done)

#------------------------------------------------------------------------------

    def create_init_msg(self, vehtype, name):
        """ Create USARSim command to spawn the robot and get configuration information. """
               
        self.tf_listener.waitForTransform(self.usarsim_frame, 
                                      self.odom_frame, 
                                      rospy.Time(), 
                                      rospy.Duration(60))
        
        # Transform ROS (odom) pose to USARSim
        pose_usar = self.trans_odom_msg_pose(self.usarsim_frame, self.ground_truth)
        q = pose_usar.pose.orientation        
        rot_usar = euler_from_quaternion([q.x, q.y, q.z, q.w])               
        
        # Spawn spawn the robot     
        msg = ('INIT {{ClassName USARBot.{0}}} '.format(vehtype)
            + '{{Name {0}}} '.format(name) 
            + '{{Location {0},{1},{2}}} '.format(pose_usar.pose.position.x, 
                                                pose_usar.pose.position.y, 
                                                pose_usar.pose.position.z)
            + '{{Rotation {0},{1},{2}}}\r\n'.format(rot_usar[0]+pi, rot_usar[1], rot_usar[2]))
        # HACK: Roll angle increased by pi, because USARSim seems to be
        # handling initial rotation falsly
        
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
        # TODO: Laser location and orientation should actually also be
        # properly transformed!
        # Code below will likely fail for a laser that's not looking straight ahead       
        for (name,par) in args.items():
            if name == 'Orientation':
                rot = [float(s) for s in par.split(',')]
                self.laser_tf.transform.rotation = Quaternion(*quaternion_from_euler(*rot))
            elif name == 'Location':
                loc = [float(s) for s in par.split(',')]
                self.laser_tf.transform.translation = Vector3(loc[0],loc[1],-loc[2])

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
                                          self.base_link_frame,
                                          self.odom_frame)            
            for (name, par) in args.items():
                par = par.split(',')            
                if name == 'Vel':
                    w_left = float(par[0])
                    w_right = float(par[1])
                    lin_vel = (w_left + w_right) * self.r / 2.0
                    rot_vel = self.r * (w_right - w_left) / self.l
                    self.odom.twist.twist.linear.x = lin_vel        
                    self.odom.twist.twist.angular.z = -rot_vel
            self.odom.header.stamp = time_now
            # Publish the odom message
            self.odom_pub.publish(self.odom)
        else:
            rospy.logwarn("Can't compute velocity, robot parameters unknown!")    
#------------------------------------------------------------------------------

    def handle_scanner_msg(self, args):
        # Broadcast scanner transform first
        time_now = rospy.Time.now()        
        pos = self.laser_tf.transform.translation
        rot = self.laser_tf.transform.rotation        
        self.tf_broadcaster.sendTransform((pos.x, pos.y, pos.z),
                                      (rot.x, rot.y, rot.z, rot.w),
                                      time_now,
                                      self.laser_tf.child_frame_id,
                                      self.laser_tf.header.frame_id)
        
        scan = LaserScan()
        scan.header.stamp = time_now
        scan.header.frame_id = self.base_laser_frame                            
        for (name, par) in args.items():
            if name == 'Range':
                scan.ranges = [float(s) for s in par.split(',')]
            if name == 'FOV':
                fov = float(par)
                scan.angle_min = -fov/2.0
                scan.angle_max = fov/2.0
            if name == 'Resolution':
                scan.angle_increment = float(par)
        scan.range_min = 0.0
        scan.range_max = 20.0

        self.scan_pub.publish(scan)

#------------------------------------------------------------------------------

    def handle_odometry_msg(self, args):
        for (name, par) in args.items():
            if name == 'Pose':
                # Unpack data sent by usarsim                
                time_now = rospy.Time.now()                    
                xyyaw = [float(s) for s in par.split(',')]
                self.odom.pose.pose.position = Point(xyyaw[0], xyyaw[1], 0.0)
                # Add pi roll, because USARSim Z-axis is pointing downwards
                # (supposedly %)                
                q = tf.transformations.quaternion_from_euler(pi, 0.0, xyyaw[2])
                self.odom.pose.pose.orientation = Quaternion(*q)
                self.odom.header.frame_id = self.usarsim_frame
                # Transform to ROS (odom) cordinates
                odom_pose = self.trans_odom_msg_pose(self.odom_frame, self.odom)                
                odom_pose.pose.position.z = 0.0 # Odometry doesn't fly :)
                # We don't know how USARSim coordinates are set up on a particular map
                # This way, we make sure our odometry is always on the ground                
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
        yaw_old = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]        

        for (name, par) in args.items():            
            if name == 'Location':
                p = [float(s) for s in par.split(',')]        
                self.ground_truth.pose.pose.position = Point(*p)                
            elif name == 'Orientation':
                rot = [float(s) for s in par.split(',')]
                # Adding pi to roll, because USARSim is reporting it incorrectily        
                q = quaternion_from_euler(rot[0]+pi, rot[1], rot[2])        
                self.ground_truth.pose.pose.orientation = Quaternion(*q)
        
        # Transform from USARSim to ROS (odom) coordinates                
        self.ground_truth.header.frame_id = self.usarsim_frame        
        truth_pose = self.trans_odom_msg_pose(self.odom_frame, self.ground_truth)                
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

    def trans_odom_msg_pose(self, target_frame, odom_msg, ts=rospy.Time()):
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

