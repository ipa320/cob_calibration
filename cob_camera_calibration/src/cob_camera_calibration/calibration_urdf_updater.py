#!/usr/bin/env python

from xml.dom import minidom

class CalibrationUrdfUpdater():
    '''
    Parses calibration.urdf.xarco and provides method to update it
    '''
    
    def __init__(self, urdf_in, urdf_out, debug=False):
        '''
        Init object with paths to input and output xml
        '''
        self.file_urdf_in =  urdf_in
        self.file_urdf_out = urdf_out
        self.debug = debug
    
    def update(self, attributes2update):
        '''
        Read urdf xml file (self.file_urdf_in) and replace values according to attributes2update dictionary.
        Save results to urdf xml file (self.file_urdf_out)
        
        @param attributes2update: names of arguments (of property tags) in urdf xml file which need to be updated -> new values
        @type  attributes2update: dictionary
        '''
        # parse xml file to dom
        print "--> loading calibration xml file from '%s'" % self.file_urdf_in
        xml_dom = minidom.parse(file(self.file_urdf_in))
        
        # get all property elements
        for node in xml_dom.getElementsByTagName("property"):
            # sanity check, all property elements need name and value attributes
            assert node.hasAttribute("name") and node.hasAttribute("value")
            attr_name  = node.getAttribute("name")
            attr_value = node.getAttribute("value")    
            
            # if attributes name is in attributes2update dict, update attributes value to new value (attributes2update[name])
            if attr_name in attributes2update:
                attr_value_new = attributes2update[attr_name]
                node.setAttribute("value", str(attr_value_new))
                if self.debug:
                    print "Updating '%s' from '%s' to '%s'" % (attr_name, attr_value, attr_value_new)
    
        # convert changed dom to pretty xml string
        xml_string = xml_dom.toprettyxml(indent="", newl="")
        
        # save string to xml_out
        print "--> saving results to calibration xml file '%s'" % self.file_urdf_out
        f = open(self.file_urdf_out, "w")
        f.writelines(xml_string) 
        f.close()
