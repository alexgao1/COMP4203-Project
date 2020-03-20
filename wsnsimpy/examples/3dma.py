import copy
import math
import random
import sys
import wsnsimpy.wsnsimpy_tk as wsp

#SOURCE = 1
# Changing DEST to 1 and everything else will be a source
DEST   = 1

# Network Parameters
NODE_TX_RANGE = 250
AREA_LENGTH = 500
AREA_WIDTH = 600
AREA_HEIGHT = 500
ENERGY_ELEC = 50 #(NANOJOULES PER BIT)
ENERGY_AMP = 100 #(PICOJOULE PER BIT PER SQUARE METER) - ???

TERRAIN_SIZE = (AREA_LENGTH, AREA_WIDTH)
TERRAIN_SIZE_WITH_Z = (AREA_LENGTH, AREA_WIDTH, AREA_HEIGHT)

# Global container for all nodes
ALL_NODES = []

#Global Statistics
stats_3dma = {
    'ete_throughputs': [],
    'ete_net_throughput': -1,
    'ete_delay': [],
    'path_lengths': [],
    'avg_path_length': -1,
    'avg_energy_consumption': -1
}

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
        if msg == 'data':
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
            if dist <= NODE_TX_RANGE:
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
        # Third value is implied to be: second_value < third_value < NODE_TX_RANGE
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
                with self.resource.request() as req:
                    # Once the resource is open it will send data
                    yield req
                    yield self.timeout(1)
                self.log(f"{self.id} wants to send data to the base node!")
                # Send data to the node in the path
                self.send_data(self.id, self.path[1::])
            else:
                yield self.timeout(1)

    ###################
    def send_data(self,src, path):
        next_node = path[0]
        self.log(f"Forward data to {next_node.id}! (Origin: {src})")
        self.send2(path, msg='data', src=src)

    ###################
    def on_receive(self, sender, path, msg, src, **kwargs):
        # Take the next node out of the path and forward packet with the new path
        next_node = path.pop(0)
        if self == next_node:
            yield self.timeout(.2)
            self.send_data(src, path, **kwargs)

###########################################################
sim = wsp.Simulator(
        until=100,
        timescale=1,
        visual=True,
        terrain_size=TERRAIN_SIZE,
        title="3DMA Demo")

# define a line style for parent links
sim.scene.linestyle("parent", color=(0,.8,0), arrow="tail", width=2)

# place nodes over 100x100 grids
# BaseNode = sim.add_node(MyNode, (5,5,5))
# BaseNode.tx_range = NODE_TX_RANGE
# BaseNode.logging = True

prevCoords = (40, 40, 40)
BaseNode = sim.add_node(BaseNode, prevCoords)
BaseNode.logging = True
ALL_NODES.append(BaseNode)
max_nodes = 125
for numNodes in range(1, max_nodes + 1):
    prevCoords = gen_within_range(prevCoords, NODE_TX_RANGE)
    node = sim.add_node(SensorNode, prevCoords)
    node.tx_range = NODE_TX_RANGE
    node.logging = True
    ALL_NODES.append(node)

for node in ALL_NODES:
    node.init()

# Initial statistics calculation
for node in ALL_NODES:
    # First node added is always the base node
    if node.id == 0:
        continue
    stats_3dma['ete_throughputs'].append(node.calculate_throughput(node.path))
    stats_3dma['path_lengths'].append(len(node.path))

stats_3dma['ete_net_throughput'] = sum(stats_3dma['ete_throughputs'])
stats_3dma['avg_path_length'] = float(sum(stats_3dma['path_lengths']) / len(stats_3dma['path_lengths']))

# start the simulation
sim.run()
