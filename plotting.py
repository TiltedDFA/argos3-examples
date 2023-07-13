import matplotlib.pyplot as plt


file_data = dict()
with open("aggregation.txt", "r") as file:
    for line in file:
        data = line.split(',')
        data[5] = data[5][:-1]
        data[1] = data[1][2:]
        if data[0] not in file_data:
            file_data[data[0]] = list()
        #data@Timestep = {FootBotId,HopCount,ForgettingActive,NumConnections,InAZone}
        file_data[data[0]].append((int(data[1]),int(data[2]),data[3] == 'true',int(data[4]),data[5] == 'true'))

for key in file_data:
    file_data[key].sort(key=lambda tup: tup[0])

