import os
import re
import subprocess as sproc

FORGETTING_TP = (1000,500,0) #set to 0 to disable
NUM_BOTS = (20,) 
IN_RANGE_RADIUS = (2,) 
PACKET_DROP_PROB = (0,0.5)
NOISE = (0,0.5)
DELAY_PROB = (0,0.5)
DELAY_STEPS = (20,100,500)

PATH_TO_WORKING_DIR = "/home/malik/Desktop/argos3-examples"

RUNNER_NAME = "automation.py"
PLOTTER_NAME = "plotting.py"

automation_args = list()
plotting_args = list()
target_folder_name = list()
num_total_runs = 0
dirs = list()


os.chdir(PATH_TO_WORKING_DIR)


for timep in FORGETTING_TP:
    for bots in NUM_BOTS:
        for radii in IN_RANGE_RADIUS:
            for probs in PACKET_DROP_PROB:
                for std_devs in NOISE:
                    for steps in DELAY_STEPS:
                        for delays in DELAY_PROB:
                            num_total_runs += 1
                            automation_arg_list = ["python", RUNNER_NAME, f"-bots={bots}"]
                            automation_arg_list.append(f"-forgetting={timep}")
                            automation_arg_list.append(f"-in-range-rad={radii}")
                            automation_arg_list.append(f"-packet-drop={probs}")
                            automation_arg_list.append(f"-noise={std_devs}")
                            automation_arg_list.append(f"-delayed-prob={delays}")
                            automation_arg_list.append(f"-delayed-steps={steps}")
                            automation_arg_list.append(f"-folder-gen-number={num_total_runs}")
                            plotting_args.append(["python", PLOTTER_NAME, f"-folder-gen-number={num_total_runs}"])
                            target_folder_name.append(f"alt_xmls{num_total_runs}/")
                            automation_args.append(automation_arg_list)

print(len(automation_args))

for i in range(0,len(automation_args)):

    process = sproc.Popen(automation_args[i])
    process.wait()

    process = sproc.Popen(plotting_args[i])
    process.wait()

    #os.system(f"tar cvf archieves/a{i} --use-compress-program='gzip -9 {target_folder_name[i]}")
    process = sproc.Popen(["tar", "-cvf", f"archieves/a{i+1}.tar.gz", "--use-compress-program=gzip -9", target_folder_name[i]])
    process.wait()
    
    file_msg = ""
    for z in range(7):
        file_msg += f"{automation_args[i][2+z][1:]}, "
    file_msg = file_msg[:-2]

    with open("archieves/info.txt", "a") as file:
        file.write(f"a{i+1} == {file_msg}\n")
    
    process = sproc.Popen(["rm", "-r", target_folder_name[i]])
    process.wait()

    
    




