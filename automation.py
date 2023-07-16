import os as sys
import xml.etree.ElementTree as ET

path_to_working_dir = "/home/malik/Documents/argos3-examples"
run_argos = "argos3 -c"
xml_location = "experiments/aggregation_one.argos"
modified_xml_folder_name = "alt_xmls"

sys.chdir(path_to_working_dir)
if sys.path.exists(modified_xml_folder_name):
    pass
else:
    sys.mkdir(modified_xml_folder_name)

tree = ET.parse(xml_location)
root = tree.getroot()

rnd_seed_node = root.find('framework').find('experiment')
fb_params_node = root.find('controllers').find('footbot_aggregation_one').find('params')
entity_node = root.find('arena').find('distribute').find('entity')

entity_node.set('quantity',str(10))

sys.system(f"{run_argos} {xml_location} -z")

