"""
    Utilities for parsing configuration files.
"""

from ConfigParser import RawConfigParser

class usar_config_parser(RawConfigParser):
    """
        Parse configuration files for USARSim/ROS interface.

        Sample usage:
        -------------

        >>> from usar_config_parser import usar_config_parser
        >>> cfg = usar_config_parser()
        >>> cfg.readfp(open('file.cfg'))
        >>> vehicles = cfg.parsed_items('vehicles')
    """
    def __init__(self):
        RawConfigParser.__init__(self)
        self.optionxform = str # Preserve uppercase letters

        # Here, we specify the fields (names and datatpyes) that are packed
        # under a single name in a particular section
        self.vehicle_fields = (('type',str), ('hostname',str), ('port',int),
                               ('x',float), ('y',float), ('z',float),
                               ('roll',float), ('pitch',float), ('yaw',float))
        self.loader_fields = (('type',str), ('hostname',str), ('port',int),
                               ('x',float), ('y',float), ('yaw',float))
        self.unloader_fields = (('type',str), ('hostname',str), ('port',int),
                                 ('x',float), ('y',float), ('yaw',float))
        
    def parsed_items(self, section):
        """
            Get the parsed items of an USARSim/ROS config file section.

            Input:
            ------
            section - Section name (string)

            Output:
            -------
            paritems - a dictionary containing the parsed items, or a dictionary of
                       dictionaries (depending on the input argument) 

            NOTE: Loaders and unloaders are incomplete. A lot of relevant parameters
            such as initial, pickup, dropoff and buffer configurations are undefined.
        """
        
        items_str = self.items(section)
        parseditems = []
        if section == 'vehicles':
            parseditems = self.parse_packed(items_str, self.vehicle_fields)
        elif section == 'loaders':
            parseditems = self.parse_packed(items_str, self.loader_fields)
        elif section == 'unloaders':
            parseditems = self.parse_packed(items_str, self.unloader_fields)
            
        return parseditems

    def parse_packed(self, items_str, fields):
        """
            Utility function, performs simple parsing of packed parameters
            (multiple values packed under one name).
        """
        parseditems = dict()
        for (name, par) in items_str:
            item = dict()
            params = par.split(';')
            for (par_str, (par_name, par_type)) in zip(params, fields):
                item[par_name] = par_type(par_str.strip())    
            parseditems[name] = item

        return parseditems
        

    def section_items(self, section):
        """
            Get all the items from a section, and their associated values.
        """
        items = dict()        
        for (name, par) in self.items(section):
            items[name] = par

        # Do some item-specific processing            
        if section == 'usarsim':
            items['port'] = int(items['port'])

        return items

