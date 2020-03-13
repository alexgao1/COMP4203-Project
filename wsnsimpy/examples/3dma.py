import random
import sys
import wsnsimpy.wsnsimpy_tk as wsp

# SOURCE = 1
# Changing DEST to 1 and everything else will be a source
DEST   = 1

NODE_TX_RANGE = 100
TERRAIN_SIZE = (700,700)
TERRAIN_SIZE_WITH_Z = (700,700,500)

# Global container for all nodes
ALL_NODES = []

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
class base_node(wsp.Node):

    ###################
    # Is only destination
    def run(self):
        # if self.id is SOURCE:
        #     self.scene.nodecolor(self.id,0,0,1)
        #     self.scene.nodewidth(self.id,2)
        #     yield self.timeout(1)
        #     self.send_rreq(self.id)
        # elif self.id is DEST:
        self.scene.nodecolor(self.id,1,0,0)
        self.scene.nodewidth(self.id,2)
        # else:
        #     self.scene.nodecolor(self.id,.7,.7,.7)

    ###################
    def on_receive(self, sender, msg, src, **kwargs):
        # When receiving data log it
        if msg == 'data':
            seq = kwargs['seq']
            self.log(f"Got data from {src} with seq {seq}")


###########################################################
class sensor_node(wsp.Node):

    ###################
    def init(self):
        super().init()
        self.prev = None

    ###################
    def run(self):
        # I do not know which color should be picked
        # if self.id is SOURCE:
        #     self.scene.nodecolor(self.id,0,0,1)
        #     self.scene.nodewidth(self.id,2)
        #     yield self.timeout(1)
        #     self.send_rreq(self.id)
        # elif self.id is DEST:
        #     self.scene.nodecolor(self.id,1,0,0)
        #     self.scene.nodewidth(self.id,2)
        # else:
        self.scene.nodecolor(self.id,.7,.7,.7)

    ###################
    # 3DMA routing protocol

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
        if msg == 'data':
            if self.id is not DEST:
                yield self.timeout(.2)
                self.send_data(src,**kwargs)
            else:
                seq = kwargs['seq']
                self.log(f"Got data from {src} with seq {seq}")

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
# base_node = sim.add_node(MyNode, (5,5,5))
# base_node.tx_range = NODE_TX_RANGE
# base_node.logging = True

prevCoords = (40, 40, 40)
base_node = sim.add_node(base_node, prevCoords)
base_node.logging = True
ALL_NODES.append(base_node)       
for numNodes in range(1, 126):
    prevCoords = gen_within_range(prevCoords, NODE_TX_RANGE)
    node = sim.add_node(sensor_node, prevCoords)
    node.tx_range = NODE_TX_RANGE
    node.logging = True
    ALL_NODES.append(node)
    
# start the simulation
sim.run()
