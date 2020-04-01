from beautifultable import BeautifulTable
import copy
import math
import random
import sys
import wsnsimpy.wsnsimpy_tk as wsp
import simpy

#SOURCE = 1
# Changing DEST to 1 and everything else will be a source
DEST   = 1

# Network Parameters
node_tx_range = 125
max_nodes = 50

AREA_LENGTH = 800
AREA_WIDTH = 700
AREA_HEIGHT = 500
ENERGY_ELEC = 50 #(NANOJOULES PER BIT)
ENERGY_AMP = 100 #(PICOJOULE PER BIT PER SQUARE METER) - ??? unused
PACKET_SIZE = 8000000 # BITS
NANOJ_TO_JOULE = 1000000000

TERRAIN_SIZE = (AREA_LENGTH, AREA_WIDTH)
TERRAIN_SIZE_WITH_Z = (AREA_LENGTH, AREA_WIDTH, AREA_HEIGHT)

# Global container for all nodes
ALL_NODES = []

#Global Statistics
stats_3dma = {
    'ete_throughputs': [],
    'ete_net_throughput': -1,
    'ete_delay': [],
    'ete_net_delay': -1,
    'path_lengths': [],
    'avg_path_length': -1,
    'indiv_energy_consumption': [],
    'avg_energy_consumption': -1
}

###########################################################
def delay():
    return random.uniform(.2,.8)

###########################################################
def is_point_in_sphere(center, range, checkPoint):
    return (((checkPoint[0] - center[0])**2 + (checkPoint[1] - center[1])**2 +
            (checkPoint[2] - center[2])**2) < range**2)

###########################################################
def gen_rand_sphere_point(pos, dimension, range):
    return random.uniform(pos[dimension] - range, pos[dimension] + range + 1)

###########################################################
def gen_within_range(pos, range):
    while True:
        newX = newY = newZ = sys.maxsize
        while newX > TERRAIN_SIZE_WITH_Z[0] or newX < 0:
            newX = gen_rand_sphere_point(pos, 0, range)
        while newY > TERRAIN_SIZE_WITH_Z[1] or newY < 0:
            newY = gen_rand_sphere_point(pos, 1, range)
        while newZ > TERRAIN_SIZE_WITH_Z[2] or newZ < 0:
            newZ = gen_rand_sphere_point(pos, 2, range)
        if is_point_in_sphere(pos, range, (newX, newY, newZ)):
            return (newX, newY, newZ)

###########################################################
def distance(pos1,pos2):
    return ((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)**0.5

###########################################################
def vector(pos1,pos2):
    return (pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2])

###########################################################
def euclid_norm(vec):
    # Square root of the sum of squares for the vector
    return sum(p**2 for p in vec)**0.5

###########################################################
# Calculate the angle between 2 vectors
def angle(vec1, vec2):
    # dot product
    dp = sum(p*q for p,q in zip(vec1,vec2))
    den = euclid_norm(vec1) * euclid_norm(vec2)
    return math.acos(dp/den)

###########################################################
class BaseNode(wsp.Node):

    ###################
    # Is only destination
    def run(self):
        self.scene.nodecolor(self.id,1,0,0)
        self.scene.nodewidth(self.id,2)

    ###################
    # Receives request of sending node
    def send_req(self, req):
        self.req = req

    ###################
    def on_receive(self, sender, path, msg, src, **kwargs):
        # When receiving data log it
        stats_3dma['ete_delay'].append(msg)
        self.log(f"Got data from {src}!")
        # Release request to allow other nodes to start sending
        self.resource.release(self.req)

###########################################################
class SensorNode(wsp.Node):

    ###################
    def init(self):
        super().init()
        self.prev = None
        # Path to BS
        self.path = []
        # Nodes that have been traversed
        self.traversed_nodes = []
        self.throughput = self.routing_3dma_ds()
        self.energy_used = 0

    ###################
    def run(self):
        # I do not know which color should be picked
        self.scene.nodecolor(self.id,.7,.7,.7)
        self.log("Start sending data.")
        # Trigger an event of actually sending data
        self.start_process(self.start_send_data())

    ###################
    # Find all neighbours within transmission range
    def find_neighbours(self, IM, parent_node = None):
        neighbour_list = []
        node_search_space = copy.copy(ALL_NODES)
        # Remove all nodes previously traversed to
        for node in self.traversed_nodes:
            node_search_space.remove(node)
        # If parent node exists and it is still in search space
        if parent_node and parent_node in node_search_space:
            node_search_space.remove(parent_node)

        for node in node_search_space:
            # If within range then it's a neighbour
            dist = distance(IM.pos, node.pos)
            if dist <= node_tx_range:
                neighbour_list.append(node)

        return neighbour_list

    ###################
    # VNP Handling
    def vnp_handling(self, IM):
        top = len(self.path)
        # Loop required for going back up path
        while True:
            # If parent node is sender then pick a new and different neighbour
            if self.path[top] == self:
                # Create neighbour list from sender. Current IM is used as a parent_node argument so it is excluded from the new neighbour list
                neighbour_list = self.find_neighbours(self, IM)
                angle_list = self.calculate_angle(neighbour_list, self)
                return self.minimum_angle_node(angle_list)
            # If parent has no other alternatives then remove the end of the path
            if len(self.find_neighbours(self.path[top]) == 1):
                del self.path[top]
                top -= 1
                continue
            # If alternative exists
            if len(self.find_neighbours(self.path[top], IM)) >= 1:
                # Select next best angle
                neighbour_list = self.find_neighbours(self, IM)
                angle_list = self.calculate_angle(neighbour_list, self)
                return self.minimum_angle_node(angle_list)
            else:
                print("Temp: unspecified condition, exit vnp_handling().")
                break

    ###################
    # Calculate Angle
    def calculate_angle(self, neighbour_list, IM):
        angle_list = []
        dest = ALL_NODES[0]
        for node in neighbour_list:
            # Turns out the angle function sometimes doesn't like the scenario of trying to find the angle between the dest node and the dest node
            if node == dest:
                angle_list.append(0)
                continue
            # Find vectors
            vec1 = vector(IM.pos, node.pos)
            vec2 = vector(IM.pos, dest.pos)
            # Calculate angle between 2 vectors
            theta = angle(vec1, vec2)
            angle_list.append(theta)

        # Need to return angles with their corresponding neighbours
        return zip(angle_list, neighbour_list)

    ###################
    # Minimum Angle Node
    def minimum_angle_node(self, angle_list):
        # Sorts neighbour_list using the angle_list. See zip in return statement from calculate_angle to get a better understanding of why this works
        a = [angle for _,angle in angle_list]
        a.sort()
        return a[0]

    ###################
    # Calculate Throughput
    def calculate_throughput(self, path):
        f_size = 1                   #Packet Size is in MB
        total_time = 0
        code_rate = None
        first_value = 150
        second_value = 200
        # Third value is implied to be: second_value < third_value < node_tx_range
        for i in range(len(self.path) - 1):
            path1 = self.path[i]
            path2 = self.path[i+1]
            # Calculate distance
            dist = distance(path1.pos, path2.pos)
            if dist <= first_value:
                code_rate = 0.75
                bit_rate = 11
            elif first_value > 150 and second_value <= 200:
                code_rate = 0.5
                bit_rate = 5.5
            else:
                code_rate = 0.5
                bit_rate = 2

            time = (f_size/code_rate) * (1/bit_rate)
            total_time += time

        return f_size/total_time

    ###################
    # 3DMA routing protocol
    def routing_3dma_ds(self):
        IM = self
        parent_node = None
        # Base station
        dest = ALL_NODES[0]
        # Add itself to the path lsit
        self.path.append(IM)
        self.traversed_nodes.append(IM)
        while IM != dest:
            # Find neighbours
            neighbour_list = self.find_neighbours(IM, parent_node)
            # If neighbour list is empty then VNP handling
            if not neighbour_list:
                temp = IM
                IM = self.vnp_handling(IM)
                # Add void node to traversed list
                self.traversed_nodes.append(temp)
                continue
            parent_node = IM
            angle_list = self.calculate_angle(neighbour_list, IM)
            IM = self.minimum_angle_node(angle_list)
            self.path.append(IM)
            self.traversed_nodes.append(IM)

        # Return throughput for later use. NOT SURE WHEN and WHERE this is used.
        return self.calculate_throughput(self.path)

    ###################
    def start_send_data(self):
        while True:
            # Gives a 20% probability it will send data
            if random.random() > 0.8:
                req = self.resource.request()
                yield req
                yield self.timeout(1)
                # Gives request to the Base node
                self.path[len(self.path) - 1].send_req(req)
                self.log(f"{self.id} wants to send data to the base node!")
                # Send data to the node in the path
                self.send_data(self.id, self.path[1::], 0)
            else:
                yield self.timeout(1)

    ###################
    def send_data(self,src, path, msg):
        next_node = path[0]
        self.log(f"Forward data to {next_node.id}! (Origin: {src})")
        self.energy_used += ENERGY_ELEC * PACKET_SIZE #Easier to define our packet size in bits here as it's energy is per bit
        self.send2(path, msg=msg, src=src)

    ###################
    def on_receive(self, sender, path, msg, src, **kwargs):
        # Take the next node out of the path and forward packet with the new path
        next_node = path.pop(0)
        if self == next_node:
            yield self.timeout(.2)
            self.energy_used += ENERGY_ELEC * PACKET_SIZE
            self.send_data(src, path, msg, **kwargs)

    def finish(self):
        stats_3dma['indiv_energy_consumption'].append(self.energy_used)

###########################################################

# Receive user input for node range and count
while True:
    rangeFlag = True
    countFlag = True
    while rangeFlag:
        try:
            range_input = int(input("Input Node Transmission Range (Default: 250, Range: 25 - 500)"))
        except ValueError:
            print("Error, not a number.")
            continue
        if range_input < 25 or range_input > 500:
            print("Input not within specified range.")
            continue
        else:
            rangeFlag = False
            break
    while countFlag:
        try:
            count_input = int(input("Input Node Count (Default: 100, Range: 100 - 500)"))
        except ValueError:
            print("Error, not a number.")
            continue
        if count_input < 100 or count_input > 500:
            print("Input not within specified range.")
            continue
        else:
            countFlag = False
            break
    if not countFlag and not rangeFlag:
        break

node_tx_range = range_input
max_nodes = count_input

sim = wsp.Simulator(
        until=60,
        timescale=1,
        visual=True,
        terrain_size=TERRAIN_SIZE,
        title="3DMA Demo")

# Define a line style for parent links
sim.scene.linestyle("parent", color=(0,.8,0), arrow="tail", width=2)

prevCoords = (40, 40, 40)
BaseNode = sim.add_node(BaseNode, prevCoords)
BaseNode.logging = True
ALL_NODES.append(BaseNode)

# Generate nodes
for numNodes in range(1, max_nodes + 1):
    prevCoords = gen_within_range(prevCoords, node_tx_range)
    node = sim.add_node(SensorNode, prevCoords)
    node.tx_range = node_tx_range
    node.logging = False
    ALL_NODES.append(node)

# Initialize them, including route building
for node in ALL_NODES:
    node.init()

# Initial statistics calculation
for node in ALL_NODES:
    # First node added is always the base node
    if node.id == 0:
        continue
    stats_3dma['ete_throughputs'].append(node.calculate_throughput(node.path))
    stats_3dma['path_lengths'].append(len(node.path))

stats = BeautifulTable()
stats.numeric_precision = 8
stats.set_style(BeautifulTable.STYLE_BOX)
stats.column_headers = ["Statistic", "Value"]

stats_3dma['ete_net_throughput'] = sum(stats_3dma['ete_throughputs'])
stats_3dma['avg_path_length'] = float(sum(stats_3dma['path_lengths']) / len(stats_3dma['path_lengths']))

# Start the simulation
sim.run()

try:
    stats_3dma['avg_energy_consumption'] = (sum(stats_3dma['indiv_energy_consumption']) / len(stats_3dma['indiv_energy_consumption'])) / NANOJ_TO_JOULE
except ZeroDivisionError:
    stats_3dma['avg_energy_consumption'] = "N/A"

try:
    stats_3dma['ete_net_delay'] = sum(stats_3dma['ete_delay']) / len(stats_3dma['ete_delay'])
except ZeroDivisionError:
    stats_3dma['ete_net_delay'] = "N/A"

stats.append_row(["Node Count", max_nodes])
stats.append_row(["Node Range", node_tx_range])
stats.append_row(["Simulation Length", AREA_LENGTH])
stats.append_row(["Simulation Width", AREA_WIDTH])
stats.append_row(["Simulation Height", AREA_HEIGHT])
stats.append_row(["End-to-End Network Throughput (Mbps)", stats_3dma['ete_net_throughput']])
stats.append_row(["End-to-End Network Delay", stats_3dma['ete_net_delay']])
stats.append_row(["Average Path Length (Hops)", stats_3dma['avg_path_length']])
stats.append_row(["Average Energy Consumption (Joules)", stats_3dma['avg_energy_consumption']])

print(stats)
