import re
import os
import sys
import automation as auto 
import matplotlib.pyplot as plt


CSV_FILE_PREFIXES   = "dat_"
CSV_FILE_SUFFIXES   = ".csv"
NUM_CSVS            = auto.NUM_RUNS 
OUTPUT_GRAPH_NAME   = "Graph.svg"

def GenGraphContext() -> str:
    graph_context = f"Experiment length: {auto.EXPERMIMENT_LENGTH}s\nTarget area size: {auto.LF_RADIUS_COUNTED_WITHIN_TRGT_BOT}\n"
    graph_context += f"Stationary target bot: {auto.EP_STATIONARY_TARGET_BOT}\n"
    graph_context += f"Packet drop prob: {auto.EP_PACKET_DROP_PROB}\nDelayed transmission prob: {auto.EP_DELAYED_TRANMISSION_PROB}\n"
    graph_context += f"Number of timesteps for delayed transmission: {auto.EP_TIME_STEPS_PER_DELAY}\n"
    graph_context += f"Number of starting bots: {auto.NUM_BOTS}\nVelocity: {auto.EP_VELOCITY}\n"
    graph_context += f"Delta: {auto.EP_DELTA}\nAlpha: {auto.EP_ALPHA}\nMaximum hop count: {auto.EP_HCMAX}\n"
    graph_context += f"Forgetting enabled: {auto.EP_FORGETTING_ON}\nTime between forgetting: {auto.EP_FORGETTING_TIMEP}timesteps\n"
    return graph_context

def TotalValFromListRobos(l:list,i:int,expected_val) -> int:
    num_total = 0
    for elem in l:
        if elem[i] == expected_val:
            num_total += 1
    return num_total

def TimeSeries(all_files_dat:list):
    time = list()
    data = list()
    
    for i in range(len(all_files_dat[0].keys())):
        data.append(list())
        time.append(i)

    for d in all_files_dat: 
        current_data_index = 0
        for key, val in d.items():
            data[current_data_index].append(TotalValFromListRobos(val,4,True))
            current_data_index += 1

    plt.figure(figsize=(7, 7))
    plt.plot(time,data)
    plt.legend(labels=[f"data{i+1}" for i in range(len(all_files_dat))])
    plt.xlabel("Time step")
    plt.ylabel("Num bots in zone")
    plt.title(f"Num bots in target agent's zone at end of simulation")
    plt.text(1.05, 0.5, GenGraphContext(), fontsize=8, ha='left', va='center', transform=plt.gca().transAxes)
    plt.grid(True)
    plt.subplots_adjust(left=0.1,right=0.5,bottom=0.1,top=0.9)
    plt.tight_layout()
    plt.savefig(OUTPUT_GRAPH_NAME)
    #plt.show()

def Main() -> None:

    folder_dir = f"{auto.PATH_TO_WORKING_DIR}/alt_xmls/"

    # if len(sys.argv) == 2:
    #     folder_dir = folder_dir + re.sub(r'[-].+[=]', '', sys.argv[1]);
    os.chdir(folder_dir)

    all_files_data = list()
    for i in range(0,5):
        file_data = dict()
        with open(f"{CSV_FILE_PREFIXES}{i+1}{CSV_FILE_SUFFIXES}", "r") as file:
            for line in file:
                data = line.split(',')
                data[5] = data[5][:-1]
                data[1] = data[1][2:]
                if data[0] not in file_data:
                    file_data[data[0]] = list()
                #data@Timestep = {FootBotId,HopCount,ForgettingActive,NumConnections,InAZone}
                file_data[data[0]].append((int(data[1]),int(data[2]),data[3] == 'true',int(data[4]),data[5] == 'true'))
        all_files_data.append(file_data)

    # for i in range(0,NUM_CSVS):
    #    random_seeds.append(auto.STARTING_RND_SEED + i)

    # for i in range(0,NUM_CSVS):
    #    random_seeds.append(i) #timestep

    # for i in all_files_data:
    #     #print(i)
    #     num_in_range = 0
    #     for key in i:
    #         print(key)
    #         if i[key][4] == True:
    #             num_in_range += 1
    #     num_robots_in_zones_at_end.append(num_in_range)
    time = list()
    data = list()
    

    for i in range(len(all_files_data[0].keys())):
        data.append(list())
        time.append(i)

    for d in all_files_data:
        current_data_index = 0
        for key, val in d.items():
            data[current_data_index].append(TotalValFromListRobos(val,4,True))
            current_data_index += 1

    # for file_data in all_files_data:
    #     total_robots_at_end = 0
    #     for data in file_data[str(len(file_data))]:
    #         total_robots_at_end += 1 if data[4] != 0 else 0
    #     num_robots_in_zones_at_end.append(total_robots_at_end)
 
    # plt.figure(figsize=(7, 7))
    # plt.plot(time,data)
    # # plt.bar(time,data)
    # plt.xlabel("Time step")OUTPUT_GRAPH_NAME
    # plt.subplots_adjust(left=0.1,right=0.5,bottom=0.1,top=0.9)
    # plt.tight_layout()
    # plt.savefig("graph.svg")
    # plt.show()
    TimeSeries(all_files_data)
if __name__ == "__main__":
    if len(sys.argv) == 2 and re.compile(r"-name=.+").match(sys.argv[1]):
        OUTPUT_GRAPH_NAME = sys.argv[1][6:] + ".svg"
    Main()
    print("Successfully finished")
#should have method to save image
#max med 1st 3rd percentile

# import matplotlib.pyplot as plt
# import numpy as np

# np.random.seed(0)

# random_seeds = np.arange(10)
# num_robots_in_zones_at_end = np.random.randint(0, 100, size=10)

# #some test objects
# class auto:
#     NUM_BOTS = [50]
#     EXPERMIMENT_LENGTH = 100
#     LF_AREA_SIZE = 10
#     LF_SECONDARY_AREA_OFFSET = 5
#     EP_STOP_AFTER_REACHING_TARGET_ZONE = True
#     EP_PACKET_DROP_PROB = 0.1
#     EP_NOISE_STD_DEV = 0.05
#     EP_TIME_STEPS_PER_DELAY = 3

# plt.bar(random_seeds, num_robots_in_zones_at_end)
# plt.xlabel("Random seed")
# plt.ylabel("Num bots in zone")
# plt.title(f"Number of bots in a zone with {auto.NUM_BOTS[0]} starting bots")

# graph_context = f"Experiment Length: {auto.EXPERMIMENT_LENGTH}\n Target Area Size: {auto.LF_AREA_SIZE}\n"
# graph_context += f"Target Area Offset: {auto.LF_SECONDARY_AREA_OFFSET}\n Stop After Reaching Zone: {auto.EP_STOP_AFTER_REACHING_TARGET_ZONE}"
# graph_context += f"\nPacket Drop Prob: {auto.EP_PACKET_DROP_PROB * 100}%\n Delayed Transmission Prob: {auto.EP_NOISE_STD_DEV}\n"
# graph_context += f"Number of timesteps for delayed transmission: {auto.EP_TIME_STEPS_PER_DELAY}"

# plt.text(1.05, 0.5, graph_context, fontsize=8, ha='left', va='center', transform=plt.gca().transAxes)
# plt.tight_layout()
# plt.show()