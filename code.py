import pickle
from collections import defaultdict, deque
from copy import deepcopy

V = int(input("Enter the number of vertices"))
adj = [[0 for i in range(V)] for j in range(V)]
# print("Now start entering the edge distances")
for i in range(V):
    for j in range(V):
        inp = input(f"e{i+1}{j+1}->")
        if inp=="INF" or inp=="inf":
            adj[i][j]=float("inf")
        else:
            adj[i][j] = float(inp)

src={}
dest={}
battery_status={}
charging_rate={}
discharging_rate={}
Max_battery={}
avg_speed={}
K = int(input("Number of Cars"))
for i in range(K):
    src[i]=int(input(f"source node of {i+1}th car"))-1
    dest[i]=int(input(f"destination node of {i+1}th car"))-1
    battery_status[i]=float(input(f"initial battery status of {i+1}th car"))
    charging_rate[i]=float(input(f"charging rate of {i+1}th car"))
    discharging_rate[i]=float(input(f"discharging rate of {i+1}th car"))
    Max_battery[i]=float(input(f"Max battery capacity of {i+1}th car"))
    avg_speed[i]=float(input(f"average speed of {i+1}th car"))

arrival_time = [[float("inf") for i in range(K)] for i in range(V)]


class car:
    def __init__(self,id, src, dest, battery_status, charging_rate, discharging_rate, Max_battery, avg_speed,adj):
        self.id = id
        self.src = src
        self.dest = dest
        self.battery_status = battery_status
        self.charging_rate = charging_rate
        self.discharging_rate=discharging_rate
        self.Max_battery=Max_battery
        self.avg_speed=avg_speed
        self.path=[]
        self.graph = deepcopy(adj)
        for i in range(adj.shape[0]):
            for j in range(adj.shape[0]):
                if adj[i][j]!=float("inf"):
                    if (adj[i][j]/self.discharging_rate)>self.Max_battery:
                        self.graph[i][j]=float("inf")

        self.edges=[]
        for i in range(adj.shape[0]):
            temp=[]
            for j in range(adj.shape[0]):
                if self.graph[i][j]!=float('inf'):
                    temp.append(j)
            self.edges.append(temp)

    def shortest_path(self):
        initial = self.src
        shortest_paths = {initial: (None, 0)}
        current_node = self.src
        visited = set()

        while current_node != self.dest:
            visited.add(current_node)
            destinations = self.edges[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinations:
                weight = self.graph[current_node][next_node] + weight_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, weight)
                else:
                    current_shortest_weight = shortest_paths[next_node][1]
                    if current_shortest_weight > weight:
                        shortest_paths[next_node] = (current_node, weight)

            next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            # next node is the destination with the lowest weight
            current_node = min(next_destinations, key=lambda k: next_destinations[k][1])

        # Work back through destinations in shortest path

        while current_node is not None:
            self.path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # Reverse path
        self.path = self.path[::-1]
        self.distance = defaultdict(int)
        for i in range(len(self.path)-2,-1,-1):
            self.distance[self.path[i]] = self.graph[self.path[i]][self.path[i+1]] + self.distance[self.path[i+1]]


        self.min_time = defaultdict(int)
        temp_battery_status = self.battery_status
        self.charge_time_required = defaultdict(int)
        for i in range(len(self.path)):
            if(i!=0):
                temp_battery_status -= self.graph[self.path[i-1]][self.path[i]]/self.discharging_rate
            self.min_time[self.path[i]] = self.distance[self.path[i]]/self.avg_speed
            min_charge_required = self.distance[self.path[i]]/self.discharging_rate
            self.charge_time_required[self.path[i]] = 0
            if(temp_battery_status<=min(self.Max_battery, min_charge_required)):
                self.charge_time_required[self.path[i]] = (min(self.Max_battery, min_charge_required)-temp_battery_status)/self.charging_rate
                self.min_time[self.path[i]] += self.charge_time_required[self.path[i]]

            arrival_time[self.path[i]][self.id] = 0
            if(i!=0):
                arrival_time[self.path[i]][self.id] = arrival_time[self.path[i-1]][self.id]+self.min_time[self.path[i-1]]-self.min_time[self.path[i]]

    def update_parameters(self, node, waiting_time):
        self.min_time[node] += waiting_time
        update = False
        for i in range(len(self.path)):
            if(update):
                self.min_time[self.path[i]] += waiting_time
                arrival_time[self.path[i]][self.id] += waiting_time
            if(self.path[i]==node):
                update=True

all_cars = {}
for i in range(K):
    obj = car(i,src[i],dest[i],battery_status[i],charging_rate[i],discharging_rate[i],Max_battery[i],avg_speed[i],adj)
    obj.shortest_path()
    all_cars[i] = obj

# def schedule(car_list, prev_charging_car, charge_time_left_list, node):
# 	car_obj_list = [w[0] for w in car_list]
#     temp = sorted([sorted([(i, w.min_time[node] + charge_time_left_list[x.id]) for w in car_obj_list if w.id!=x.id], key=lambda x:-x[1]) for i,x in enumerate(car_obj_list)], key=lambda x:x[1])
# 	return car_obj_list[temp[0]].id

# def schedule(car_list, prev_charging_car, charge_time_left_list, conflict_node):
#     car_obj_list = [w[0] for w in car_list]
#     if len(car_obj_list)==1:
#         return car_obj_list[0].id

#     ideal_car_quant = float("inf")
#     ideal_car = None
#     for i in range(len(car_obj_list)):
#         pres_car_obj = car_obj_list[i]
#         max_ = float("-inf")
#         for j in range(len(car_obj_list)):
#             if(i!=j):
#                 max_ = max(max_, (car_obj_list[j].min_time[conflict_node]+charge_time_left_list[pres_car_obj.id]))
#         if(ideal_car_quant>max_):
#             ideal_car_quant = max_
#             ideal_car = i

#     return car_obj_list[ideal_car].id
all_cars = {}
for i in range(K):
    obj = car(i,src[i],dest[i],battery_status[i],charging_rate[i],discharging_rate[i],Max_battery[i],avg_speed[i],adj)
    obj.shortest_path()
    all_cars[i] = obj

# print("initial value of all cars")
for idx in all_cars.keys():
    # print(f"idx:{idx}")
    # print(f"all_cars[idx].min_time:{all_cars[idx].min_time}")

def schedule(cars_list, prev_charging_car, charge_time_left_list, conflict_node):
    car_obj_list = [w[0] for w in cars_list]
    if len(car_obj_list)==1:
        return car_obj_list[0].id

    # print("SCHEDULING")
    # print(f"charge_time_left_list:{charge_time_left_list}")
    # print(f"car_obj_list:{car_obj_list}")
    # print(f"[w.id for w in car_obj_list]:{[w.id for w in car_obj_list]}")
    # print(f"[w.min_time for w in car_obj_list]:{[w.min_time for w in car_obj_list]}")
    # print(f"conflict_node:{conflict_node}")
    # print([sorted([(i, w.min_time[conflict_node] + charge_time_left_list[x.id]) for j,w in enumerate(car_obj_list) if j!=i], key=lambda y:-y[1]) for i,x in enumerate(car_obj_list)])
    
    temp = sorted([sorted([(i, w.min_time[conflict_node] + charge_time_left_list[x.id]) for j,w in enumerate(car_obj_list) if j!=i], key=lambda y:-y[1]) for i,x in enumerate(car_obj_list)], key=lambda y:y[0][1])
    
    # print(temp[0][0][0])

    return temp[0][0][0]
   



# make global time list
global_time_list = []
for conflict_node in range(V):
    cars_present = [all_cars[i] for i in range(K) if arrival_time[conflict_node][i]!=float("inf")]
    charge_time_left_list = [0]*K
    for i in cars_present:
        arr_time = arrival_time[conflict_node][i.id]
        dep_time = arr_time + i.charge_time_required[conflict_node]
        charge_time_left_list[i.id] = int(i.charge_time_required[conflict_node])
        global_time_list.append([i,"arrival", conflict_node, arr_time])
        global_time_list.append([i,"departure", conflict_node, dep_time])
global_time_list.sort(key=lambda x: x[3])

# print(f"global_time_list:{global_time_list}")

cars_list_list = [[]]*V  # num nodes X num cars at node
prev_event_time_list = [0]*V  # num nodes
prev_charging_cars_list = [-1]*V  # num nodes
cntr=0
while(len(global_time_list)!=0):

    # print(f"\n\ncntr:{cntr}")
    cntr+=1
    
    event = global_time_list.pop(0)

    # print(f"global_time_list after poping:{global_time_list}")
    
    conflict_node = event[2]

    # print(f"cars_list_list before event:{cars_list_list}")
    # print(f"cars_list_list[conflict_node] before event:{cars_list_list[conflict_node]}")
    # print(f"prev_event_time_list[conflict_node] before event:{prev_event_time_list[conflict_node]}")
    # print(f"prev_charging_cars_list[conflict_node] before event:{prev_charging_cars_list[conflict_node]}")

    if(event[1]=="departure"):

        # print("departure")

        poped=False
        index_event_id = -1
        for i in range(len(cars_list_list[conflict_node])):
            if(cars_list_list[conflict_node][i][0].id==event[0].id):
                index_event_id = i
        waiting_time = event[3]- prev_event_time_list[conflict_node]
        prev_event_time_list[conflict_node] = int(event[3])

        # print(f"prev_charging_cars_list[conflict_node] as index of charge_time_left_list:{prev_charging_cars_list[conflict_node]}")

        charge_time_left_list[prev_charging_cars_list[conflict_node]] -= waiting_time
        if(prev_charging_cars_list[conflict_node]==event[0].id):
            cars_list_list[conflict_node].pop(index_event_id)
            poped=True
        
        if(len(cars_list_list[conflict_node])>0):
            for i in range(len(cars_list_list[conflict_node])):
                if(cars_list_list[conflict_node][i][0].id!=prev_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arrival", old_list[2], arr_time] if old_list[1]=="arrival" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if poped and len(cars_list_list[conflict_node])>0:
            prev_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], prev_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)

    else:

        # print("arrival")

        index_event_id = -1
        for i in range(len(cars_list_list[conflict_node])):
            if(cars_list_list[conflict_node][i][0].id==event[0].id):
                index_event_id = i
        waiting_time = event[3] - prev_event_time_list[conflict_node]
        prev_event_time_list[conflict_node] = event[3]

        # print(f"event before appending:{event}")
        # print(f"cars_list_list[conflict_node] before appending:{cars_list_list[conflict_node]}")

        cars_list_list[conflict_node].append(event)

        # print(f"cars_list_list[conflict_node] after appending:{cars_list_list[conflict_node]}")

        charge_time_left_list[prev_charging_cars_list[conflict_node]] -= waiting_time
        if(len(cars_list_list[conflict_node])>0):   # changing departure time
            for i in range(len(cars_list_list[conflict_node])):
                if(cars_list_list[conflict_node][i][0].id!=prev_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arrival", old_list[2], arr_time] if old_list[1]=="arrival" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if len(cars_list_list[conflict_node])>0:
            prev_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], prev_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)  # scheduling

    # print("\nvalue of all cars after update")
    # for idx in all_cars.keys():
    #     print(f"idx:{idx}")
    #     print(f"all_cars[idx].min_time:{all_cars[idx].min_time}")

print("\nfinal value of all cars")
for idx in all_cars.keys():
    print(f"idx:{idx}")
    print(f"all_cars[idx].min_time:{all_cars[idx].min_time}")
    