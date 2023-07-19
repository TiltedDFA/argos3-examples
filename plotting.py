import matplotlib.pyplot as plt
import os as sys

sys.chdir("alt_xmls")

file_data = dict()
with open("dat_1", "r") as file:
    for line in file:
        data = line.split(',')
        data[5] = data[5][:-1]
        data[1] = data[1][2:]
        if data[0] not in file_data:
            file_data[data[0]] = list()
        #data@Timestep = {FootBotId,HopCount,ForgettingActive,NumConnections,InAZone}
        file_data[data[0]].append((int(data[1]),int(data[2]),data[3] == 'true',int(data[4]),data[5] == 'true'))

l1 = list()
l2 = list()
for key in file_data:
    file_data[key].sort(key=lambda tup: tup[0])
for key in file_data:
    l1.append(int(key))
    l2.append(file_data[key][4])

plt.plot(l1,l2)

plt.show()

print(len(file_data))