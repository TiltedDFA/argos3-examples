import os
import shutil as copier
import xml.etree.ElementTree as ET
import time as wait
import multiprocessing as mp
import sys

#constants
RUN_ARGOS           = "argos3 -c"
XML_OUT_DIR         = "/mnt/d/coding/argos3-examples/alt_xmls"
XML_OUT_SUFFIX      = ".ModifiedArgos"
OG_XML_LOCATION     = "experiments/aggregation_one.argos"
PATH_TO_WORKING_DIR = "/mnt/d/coding/argos3-examples"
XML_OUT_PREFIX      = "agg_"
CSV_OUT_PREFIX      = "dat_"
EXPERMIMENT_LENGTH  = 60     #in seconds 
TICKS_STEPS_PER_SEC = "10"  
STARTING_RND_SEED   = 50
NUM_RUNS            = 10    #current works as a delta for the random seed 
POST_EXPERIMENT_WAIT= 1     #also in seconds(used to account for argos start up time)
NUM_BOTS            = (50,)  #can set mulitple. will rerun experiments with same 
                            #settings for num bots listed here
CSV_IN_FILE_NAME    = "aggregation.txt"
NUM_PROCESSES       = 12
#Experiment params 
EP_VELOCITY         = "5"
EP_DELTA            = "0.1"
EP_ALPHA            = "10"
EP_HCMAX            = "99"
EP_FORGETTING_ON    = "true"
EP_FORGETTING_TIMEP = "1000"
LF_DEFAULT_TRGT_AREA= "false"
LF_NUM_TARGET_AREAS = "2"
LF_AREA_SIZE        = "0.3"
LF_SECONDARY_AREA_OFFSET = "0.2"
EP_STOP_AFTER_REACHING_TARGET_ZONE = "false" #'true' or 'false'
#COM FAULTS
EP_PACKET_DROP_PROB         = "0.5"
EP_NOISE_STD_DEV            = "0.5"
EP_DELAYED_TRANMISSION_PROB = "0"
EP_TIME_STEPS_PER_DELAY     = "0"

def RunArgos(file_name:str, num:int, csv_name:str):
    os.chdir(PATH_TO_WORKING_DIR)

    os.system(f"{RUN_ARGOS} {XML_OUT_DIR}/{file_name} -z")

    wait.sleep(EXPERMIMENT_LENGTH + POST_EXPERIMENT_WAIT)

    original    = f"{PATH_TO_WORKING_DIR}/{csv_name}"

    target      = f"{XML_OUT_DIR}/{CSV_OUT_PREFIX}" + str(num+1) + ".csv"

    copier.copyfile(original,target)


if __name__ == "__main__":
    #internal var
    quit_after_gen_xml = False
    #variables
    current_xml_num     = 1
    current_rnd_seed    = STARTING_RND_SEED
    created_xml_files   = list()
    current_xml_name    = XML_OUT_PREFIX + str(current_xml_num) + XML_OUT_SUFFIX

    if len(sys.argv) == 2 and sys.argv[1] == '-xml-only':
        quit_after_gen_xml = True

    #create xml out folder
    os.chdir(PATH_TO_WORKING_DIR)

    if os.path.exists(XML_OUT_DIR):

        os.system(f"rm -r {XML_OUT_DIR}/")

        os.mkdir(XML_OUT_DIR)

    else:
        os.mkdir(XML_OUT_DIR)


    #xml setup
    tree = ET.parse(OG_XML_LOCATION)

    root = tree.getroot()

    entity_node     = root.find('arena').find('distribute').find('entity')

    experiment_node = root.find('framework').find('experiment')

    fb_params_node  = root.find('controllers').find('footbot_aggregation_one').find('params')

    rnb_node        = root.find('controllers').find('footbot_aggregation_one').find('sensors').find('range_and_bearing')

    loop_fun_params = root.find('loop_functions').find('aggregation')

    #setting up experiment params
    fb_params_node.set('Velocity',EP_VELOCITY)

    fb_params_node.set('Delta', EP_DELTA)

    fb_params_node.set('Alpha', EP_ALPHA)

    fb_params_node.set('HopCountMax', EP_HCMAX)

    fb_params_node.set('ForgettingAllowed',EP_FORGETTING_ON)

    fb_params_node.set('ForgettingTimePeriod', EP_FORGETTING_TIMEP)

    experiment_node.set('ticks_per_second',TICKS_STEPS_PER_SEC)

    loop_fun_params.set('log_as_csv', 'true')

    loop_fun_params.set('file_name', CSV_IN_FILE_NAME)

    loop_fun_params.set('default_area_config', LF_DEFAULT_TRGT_AREA)

    loop_fun_params.set('num_target_areas', LF_NUM_TARGET_AREAS)

    loop_fun_params.set('target_area_size', LF_AREA_SIZE)

    loop_fun_params.set('secondary_area_offset', LF_SECONDARY_AREA_OFFSET)

    rnb_node.set('packet_drop_prob', EP_PACKET_DROP_PROB)

    rnb_node.set('noise_std_dev', EP_NOISE_STD_DEV)

    fb_params_node.set('DelayedTransmittionProb', EP_DELAYED_TRANMISSION_PROB)

    fb_params_node.set('TimeStepsPerDelay', EP_TIME_STEPS_PER_DELAY)

    fb_params_node.set('StopAfterReachingTargetZone', EP_STOP_AFTER_REACHING_TARGET_ZONE)

    #creating desired test files
    os.chdir(XML_OUT_DIR)

    for i in range(0,NUM_RUNS):

        experiment_node.set('length', str(EXPERMIMENT_LENGTH))

        experiment_node.set('random_seed', str(int(current_rnd_seed + i)))

        for num_bots in NUM_BOTS:
            
            entity_node.set('quantity', str(num_bots))
            
            loop_fun_params.set('file_name', f"{CSV_IN_FILE_NAME}{current_xml_num}")

            tree.write(current_xml_name)

            created_xml_files.append(current_xml_name)

            current_xml_num += 1

            current_xml_name = XML_OUT_PREFIX + str(current_xml_num) + XML_OUT_SUFFIX
            
    if quit_after_gen_xml:
        sys.exit()
    #run the created xml files and record the data produced
    run_numbers = [i for i in range(0,len(created_xml_files))]
    csv_out_name = [f"{CSV_IN_FILE_NAME}{i+1}" for i in range(0,len(created_xml_files))]
    function_run_data = [(created_xml_files[i],run_numbers[i],csv_out_name[i]) for i in range(0,len(created_xml_files))]

    print(function_run_data)

    with mp.Pool(processes=NUM_PROCESSES) as pool:
        pool.starmap(RunArgos,function_run_data)

    os.chdir(PATH_TO_WORKING_DIR)
    for name in csv_out_name:
        os.system(f"rm {name}")
    
    print("Successfully finished")
