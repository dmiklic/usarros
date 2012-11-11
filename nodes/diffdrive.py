#!/usr/bin/env python

# ROS core imports
import roslib; roslib.load_manifest('usarros')

# Custom ROS imports
from usar_config_parser import usar_config_parser

# Imports from this package
from usar_client import usar_client

# Standard library imports
import sys 
import asyncore

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
        self.msg_handlers = {'Tachometer': self.handle_tacho_msg,
                           'RangeScanner': self.handle_scanner_msg,
                           'GroundTruth': self.handle_truth_msg 
                        }        
        spawn_msg = self.create_spawn_msg(vehpar, name)        
        self.client = usar_client(usarpar['hostname'], usarpar['port'], 
                        self.parse_msg, self.create_spawn_msg(vehpar, name))

    def create_spawn_msg(self, vehpar, name):
        """ Create USARSim command to spawn the robot. """
        msg = ('INIT {{ClassName USARBot.{0}}} '.format(vehpar['type'])
            + '{{Name {0}}} '.format(name) 
            + '{{Location {0},{1},{2}}} '.format(vehpar['x'], vehpar['y'], vehpar['z'])
            + '{{Rotation {0},{1},{2}}}\r\n'.format(vehpar['roll'], vehpar['pitch'], vehpar['yaw']))
        return msg

    def parse_msg(self, msg):
        """ Handles incoming USARSim messages """
        print(msg)        
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
            self.msg_handlers[args['Type']](args)

    def handle_tacho_msg(self, args):
        for (name, par) in args.items():
            print(name)

    def handle_scanner_msg(self, args):
        for (name, par) in args.items():
            print(name)

    def handle_truth_msg(self, args):
        for (name, par) in args.items():
            print(name)

    def run(self):
        """ Keep communicating with usarsim """
        asyncore.loop()

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

if __name__ == '__main__':
    
    args = parse_arguments(sys.argv)    
    vehicle = diffdrive(args['config'], args['robot'])
    vehicle.run()

