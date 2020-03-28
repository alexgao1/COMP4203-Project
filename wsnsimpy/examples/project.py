from beautifultable import BeautifulTable
import copy
import math
import random
import sys
import wsnsimpy.wsnsimpy_tk as wsp
from enum import Enum
import time
import hashlib

#SOURCE = 1
# Changing DEST to 1 and everything else will be a source
DEST   = 1

# Network Parameters
node_tx_range = 100
AREA_LENGTH = 500
AREA_WIDTH = 600
AREA_HEIGHT = 500
ENERGY_ELEC = 50 #(NANOJOULES PER BIT)
ENERGY_AMP = 100 #(PICOJOULE PER BIT PER SQUARE METER) - ???
PACKET_SIZE = 8000000 # BITS
NANOJ_TO_JOULE = 1000000000

TERRAIN_SIZE = (AREA_LENGTH, AREA_WIDTH)
TERRAIN_SIZE_WITH_Z = (AREA_LENGTH, AREA_WIDTH, AREA_HEIGHT)

# Global container for all nodes
ALL_NODES = []

#Global Statistics
stats_3dma_onama = {
    'ete_throughputs': [],
    'ete_net_throughput': -1,
    'ete_delay': [],
    'ete_net_delay': -1,
    'path_lengths': [],
    'avg_path_length': -1,
    'indiv_energy_consumption': [],
    'avg_energy_consumption': -1,
    'senders_per_mis': [],
    'mean_concurrency': -1
}

class EntityState(Enum):
    UNDECIDED = 1
    ACTIVE = 2
    INACTIVE = 3

TIME = int(time.time())

###########################################################
def delay():
    return random.uniform(.2,.8)

def is_point_in_sphere(center, range, checkPoint):
    return (((checkPoint[0] - center[0])**2 + (checkPoint[1] - center[1])**2 +
            (checkPoint[2] - center[2])**2) < range**2)

def gen_rand_sphere_point(pos, dimension, range):
    return random.uniform(pos[dimension] - range, pos[dimension] + range + 1)

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

def update_time():
    TIME = time.time()

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
class MyNode(wsp.Node):
    tx_range = 100

    ###################
    def init(self):
        super().init()
        self.prev = None

    ###################
    def run(self):
        if self.id is SOURCE:
            self.scene.nodecolor(self.id,0,0,1)
            self.scene.nodewidth(self.id,2)
            yield self.timeout(1)
            self.send_rreq(self.id)
        elif self.id is DEST:
            self.scene.nodecolor(self.id,1,0,0)
            self.scene.nodewidth(self.id,2)
        else:
            self.scene.nodecolor(self.id,.7,.7,.7)

    ###################
    def send_rreq(self,src):
        self.send(wsp.BROADCAST_ADDR, msg='rreq', src=src)

    ###################
    def send_rreply(self,src):
        if self.id is not DEST:
            self.scene.nodecolor(self.id,0,.7,0)
            self.scene.nodewidth(self.id,2)
        self.send(self.prev, msg='rreply', src=src)

    ###################
    def start_send_data(self):
        self.scene.clearlinks()
        seq = 0
        while True:
            yield self.timeout(1)
            self.log(f"Send data to {DEST} with seq {seq}")
            self.send_data(self.id, seq)
            seq += 1

    ###################
    def send_data(self,src,seq):
        self.log(f"Forward data with seq {seq} via {self.next}")
        self.send(self.next, msg='data', src=src, seq=seq)

    ###################
    def on_receive(self, sender, msg, src, **kwargs):

        if msg == 'rreq':
            if self.prev is not None: return
            self.prev = sender
            self.scene.addlink(sender,self.id,"parent")
            if self.id is DEST:
                self.log(f"Receive RREQ from {src}")
                yield self.timeout(5)
                self.log(f"Send RREP to {src}")
                self.send_rreply(self.id)
            else:
                yield self.timeout(delay())
                self.send_rreq(src)

        elif msg == 'rreply':
            self.next = sender
            if self.id is SOURCE:
                self.log(f"Receive RREP from {src}")
                yield self.timeout(5)
                self.log("Start sending data")
                self.start_process(self.start_send_data())
            else:
                yield self.timeout(.2)
                self.send_rreply(src)

        elif msg == 'data':
            if self.id is not DEST:
                yield self.timeout(.2)
                self.send_data(src,**kwargs)
            else:
                seq = kwargs['seq']
                self.log(f"Got data from {src} with seq {seq}")

###########################################################
class BaseNode(wsp.Node):

    ###################
    # Is only destination
    def run(self):
        self.scene.nodecolor(self.id,1,0,0)
        self.scene.nodewidth(self.id,2)

    ###################
    def on_receive(self, sender, path, msg, src, **kwargs):
        # When receiving data log it
        stats_3dma_onama['ete_delay'].append(msg)
        self.log(f"Got data from {src}!")

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
        self.hash = self.generate_hash()
        self.state = EntityState.UNDECIDED

    ###################
    def run(self):
        # I do not know which color should be picked
        self.scene.nodecolor(self.id,.7,.7,.7)
        self.log("Start sending data.")
        # Trigger an event of actually sending data
        self.start_process(self.start_send_data())

    def generate_hash(self):
        # First you concat the id and the time
        concat_string = str(self.id) + str(TIME)
        # Next you hash that concat, digest it in hex, convert it to base 10, cast it to a string, concat with id again, then cast it back to an int
        self.hash = int(str(int(hashlib.md5(concat_string.encode('utf-8')).hexdigest(),16)) + str(self.id))

    def get_hash(self):
        return self.hash

    def dmis(self):
        neighbour_list = self.find_neighbours(self, routeSearch=False)
        # while self.state == EntityState.UNDECIDED: #Will cause infinite loop
        # If current node's hash is greater than all neighbours
        if all([self.hash > node.hash and node.state != EntityState.INACTIVE for node in neighbour_list]):
            self.state = EntityState.ACTIVE
        elif any([node.state == EntityState.ACTIVE for node in neighbour_list]):
            self.state = EntityState.INACTIVE

    ###################
    # Find all neighbours within transmission range
    def find_neighbours(self, IM, parent_node = None, routeSearch = True):
        neighbour_list = []
        node_search_space = copy.copy(ALL_NODES)
        # If the function is used for route finding then remove nodes from search space
        if routeSearch:
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
        # dist_list = len(path)           #NOTE: Figure out if path should be reduced by one since it MIGHT include itself
        total_time = 0
        code_rate = None
        first_value = 150
        second_value = 200
        # Third value is implied to be: second_value < third_value < node_tx_range
        # for path in self.path:
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
        # self.scene.clearlinks() #It looks better without this
        while True:
            # Gives a 20% probability it will send data
            if random.random() > 0.8:
                # with self.resource.request() as req:
                    # Once the resource is open it will send data
                    # yield req
                yield self.timeout(1)
                self.log(f"{self.id} wants to send data to the base node!")
                # Send data to the node in the path
                # self.send_data(self.id, self.path[1::])
                onama_scheduler.request_to_send(self)
                break
            else:
                yield self.timeout(1)

    def success_send(self):
        self.send_data(self.id, self.path[1::], 0)
        # After sending it will collect and keep trying
        self.start_send_data()

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
        stats_3dma_onama['indiv_energy_consumption'].append(self.energy_used)

###########################################################
class ContentionEntity():
    def __init__(self, node):
        # Exclude destination node
        self.path = node.path[:-1]
        self.id = node.id
        self.hash = self.generate_hash()
        self.state = EntityState.UNDECIDED
        self.neighbours = []

    def add_neighbour(self, ent):
        self.neighbours.append(ent)

    # Remove specific neighbour
    def remove_neighbour(self, ent):
        try:
            self.neighbours.remove(ent)
        except:
            pass

    def remove_neighbour_list(self, ent_list):
        for ent in ent_list:
            self.remove_neighbour(ent)

    # Remove ALL neighbours
    def clear_neighbours(self):
        self.neighbours = []

    def generate_hash(self):
        # First you concat the id and the time
        concat_string = str(self.id) + str(TIME)
        # Next you hash that concat, digest it in hex, convert it to base 10, cast it to a string, concat with id again, then cast it back to an int
        self.hash = int(str(int(hashlib.md5(concat_string.encode('utf-8')).hexdigest(),16)) + str(self.id))

    def get_hash(self):
        return self.hash

    def dmis(self):
        # while self.state == EntityState.UNDECIDED: #Will cause infinite loop
        # If current node's hash is greater than all neighbours
        if self.state == EntityState.UNDECIDED:
            # if len(self.neighbours) < 1:
            #     print("Node " + str(self.id) + " has no neighbours")
            if all([self.hash > ent.hash and ent.state != EntityState.INACTIVE for ent in self.neighbours]):
                self.state = EntityState.ACTIVE
                # print("Node " + str(self.id) + "  \tis now ACTIVE")
            elif any([node.state == EntityState.ACTIVE for node in self.neighbours]):
                self.state = EntityState.INACTIVE
                # print("Node " + str(self.id) + "  \tis now INACTIVE")

###########################################################
# ONAMA Scheduler
class Scheduler():
    def __init__(self, res):
        # 2D list
        self.resource = res
        # self.timeout = self.sim.timeout IMPORT SIM INTO THIS CLASS CHECK LINE $# OF wsnsimpy.py *********************************
        self.queue = []
        self.MIS = []
        self.ent_to_node_pair = {}
        self.node_to_ent_pair = {}
        self.graph = []
        self.graph_vertices = []
        self.graph_edges = []
        self.entities = []
        self.timeout = sim.timeout
        for node in ALL_NODES:
            if type(node) is SensorNode:
                self.add_node(node)

    '''
    Scheduler gets initialized with ALL_NODES and stores it. Every node then gets a contention entity, dictionary is made for both
    node -> entity and entity -> node.
    When a node wants to send it requests so from the scheduler, its entity is added to the graph
    Scheduler(This class) has a timer that after every interval it creates the MIS using the entities in the graph. It also updates the global time and has every entity update its hash
    DMIS is done with the entities
    Cycle through all ACTIVE entities and allow their corresponding nodes to send their data
    Reset graph and mis lists
    Every node that didn't get to send retries while the ones that completed go through the probability again
    TODO: Bring dmis, hash and state from SensorNode into either Scheduler or ContentionEntity
    Remove just active nodes from MIS after they send
    Regenerate hashes somewhere
    '''
    def add_node(self, node):
        ent = ContentionEntity(node)
        self.entities.append(ent)
        self.ent_to_node_pair[ent] = node
        self.node_to_ent_pair[node] = ent

    def request_to_send(self, node):
        ent = self.node_to_ent_pair[node]
        self.add_to_graph(ent)

    def add_to_graph(self, ent):
        # Update entity hash when entering the graph
        ent.generate_hash()
        # From all current entities in the graph check for path overlaps
        for e in self.graph_vertices:
            # If there is overlap in paths make an edge between the two entities
            if list(set(ent.path) & set(e.path)) != []:
                # Both entities add each other as neighbours
                ent.add_neighbour(e)
                e.add_neighbour(ent)
                pair = (ent, e)
                self.graph_edges.append(pair)
        # Add it to current verticies
        self.graph_vertices.append(ent)

    def sim_loop(self):
        while True:
            # with self.resource.request() as req:
            yield self.timeout(3)
            update_time()
            # Run all nodes in MIS and clear things
            print("Running MIS")
            self.run_mis()

    def run_mis(self):
        # Make sure all entities are not UNDECIDED
        self.set_dmis()
        # Add all active entities to the MIS
        self.mis()
        # Run all ACTIVE nodes
        for active_ent in self.MIS:
            self.ent_to_node_pair[active_ent].success_send()

        # Remove active entities
        removed_ents = []
        for ent in self.MIS:
            # Remove vertices
            self.graph_vertices.remove(ent)
            # Remove edges. filter removes any instance that satisfies the condition of the first argument.
            # Any edge tuple that contains an active entry is filtered out
            self.graph_edges = list(filter(lambda x: x[0] != ent and x[1] != ent, self.graph_edges))
            ent.clear_neighbours()
            removed_ents.append(ent)

        print("Nodes sent: ", [x.id for x in removed_ents])
        stats_3dma_onama['senders_per_mis'].append([x.id for x in removed_ents])

        # Reset entity states of left over nodes
        for ent in self.graph_vertices:
            ent.state = EntityState.UNDECIDED
            # Regenerate hash since they won't be readded
            ent.generate_hash()
            # Remove
            ent.remove_neighbour_list(removed_ents)
        # Empty MIS
        self.MIS = []

    def set_dmis(self):
        undecided = True
        # Sets every nodes state until none left are undecided
        while undecided:
            for node in self.graph_vertices:
                node.dmis()
            undecided = self.remaining_undecided()

    def mis(self):
        # Add all active nodes to MIS
        for ent in self.graph_vertices:
            if ent.state == EntityState.ACTIVE:
                self.MIS.append(ent)

    def remaining_undecided(self):
        # Returns true if all node states are NOT undecided
        return not all([node.state != EntityState.UNDECIDED for node in self.graph_vertices])

###########################################################

# Receive user input for node range and count
# while True:
    # rangeFlag = True
    # countFlag = True
    # while rangeFlag:
        # try:
            # range_input = int(input("Input Node Transmission Range (Default: 250, Range: 25 - 500)"))
        # except ValueError:
            # print("Error, not a number.")
            # continue
        # if range_input < 25 or range_input > 1300:
            # print("Input not within specified range.")
            # continue
        # else:
            # rangeFlag = False
            # break
    # while countFlag:
        # try:
            # count_input = int(input("Input Node Count (Default: 100, Range: 100 - 500)"))
        # except ValueError:
            # print("Error, not a number.")
            # continue
        # if count_input < 100 or count_input > 500:
            # print("Input not within specified range.")
            # continue
        # else:
            # countFlag = False
            # break
    # if not countFlag and not rangeFlag:
        # break
        
# node_tx_range = range_input
# max_nodes = count_input

sim = wsp.Simulator(
        until=20,
        timescale=1,
        visual=True,
        terrain_size=TERRAIN_SIZE,
        title="3DMA with ONAMA Demo")

# define a line style for parent links
sim.scene.linestyle("parent", color=(0,.8,0), arrow="tail", width=2)

prevCoords = (40, 40, 40)
BaseNode = sim.add_node(BaseNode, prevCoords)
BaseNode.logging = True
ALL_NODES.append(BaseNode)
max_nodes = 125
for numNodes in range(1, max_nodes + 1):
    prevCoords = gen_within_range(prevCoords, node_tx_range)
    node = sim.add_node(SensorNode, prevCoords)
    node.tx_range = node_tx_range
    node.logging = True
    ALL_NODES.append(node)

for node in ALL_NODES:
    node.init()

onama_scheduler = Scheduler(sim.resource)
sim.add_scheduler(onama_scheduler)

# Initial statistics calculation
for node in ALL_NODES:
    # First node added is always the base node
    if node.id == 0:
        continue
    stats_3dma_onama['ete_throughputs'].append(node.calculate_throughput(node.path))
    stats_3dma_onama['path_lengths'].append(len(node.path))
stats = BeautifulTable()
stats.column_headers = ["Statistic", "Value"]
stats_3dma_onama['ete_net_throughput'] = sum(stats_3dma_onama['ete_throughputs'])
stats_3dma_onama['avg_path_length'] = float(sum(stats_3dma_onama['path_lengths']) / len(stats_3dma_onama['path_lengths']))

# start the simulation
sim.run()

try:
    stats_3dma_onama['avg_energy_consumption'] = (sum(stats_3dma_onama['indiv_energy_consumption']) / len(stats_3dma_onama['indiv_energy_consumption'])) / NANOJ_TO_JOULE
except ZeroDivisionError:
    stats_3dma_onama['avg_energy_consumption'] = "N/A"
    
try:
    stats_3dma_onama['ete_net_delay'] = sum(stats_3dma_onama['ete_delay']) / len(stats_3dma_onama['ete_delay'])
except ZeroDivisionError:
    stats_3dma_onama['ete_net_delay'] = "N/A"
    
try:
    stats_3dma_onama['mean_concurrency'] = sum([len(slot) for slot in stats_3dma_onama['senders_per_mis']]) / len(stats_3dma_onama['senders_per_mis'])
except ZeroDivisionError:
    stats_3dma_onama['mean_concurrency'] = "N/A"

stats.append_row(["Node Count", max_nodes])
stats.append_row(["Node Range", node_tx_range])
stats.append_row(["Simulation Length", AREA_LENGTH])
stats.append_row(["Simulation Width", AREA_WIDTH])
stats.append_row(["Simulation Height", AREA_HEIGHT])
stats.append_row(["End-to-End Network Throughput (Mbps)", stats_3dma_onama['ete_net_throughput']])
stats.append_row(["End-to-End Network Delay (Seconds)", stats_3dma_onama['ete_net_delay']])
stats.append_row(["Average Path Length (Hops)", stats_3dma_onama['avg_path_length']])
stats.append_row(["Average Energy Consumption (Joules)", stats_3dma_onama['avg_energy_consumption']])
stats.append_row(["Mean Concurrency", stats_3dma_onama['mean_concurrency']])

print(stats)