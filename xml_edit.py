import xml.etree.ElementTree as ET
import os as sys

working_dir = "/home/malik/Documents/argos3-examples/alt_xmls"
current_xml_file = "aggregation_one.xml"

FOOTBOT_QUANITIES = (3,5,7)


sys.chdir(working_dir)
tree = ET.parse(current_xml_file)
root = tree.getroot()

rnd_seed_node = root.find('framework').find('experiment')
fb_params_node = root.find('controllers').find('footbot_aggregation_one').find('params')
entity_node = root.find('arena').find('distribute').find('entity')

entity_node.set('quantity',str(10))

tree.write("output.xml")

#modify random seed. 
#modify entity count.
#modify foot bot params.