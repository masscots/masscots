import os
import sys
import math
import time
import json
import asyncio
from operator import eq
from socket import *
from typing import Tuple, Union, Text
import configparser
import threading
import gym

from BioAIR.Drones.Exception import (
    ConnectionToDroneException
)

from BioAIR.Drones.State import NodeState, TentacleState, EtcState

# from mavsdk import System
# from mavsdk import (
# 		Attitude,
# 		OffboardError,
# 		PositionNedYaw,
# 		VelocityBodyYawspeed,
# 		VelocityNedYaw,
# )

Address = Tuple[str, int]

### hyper parameter
SAMPLENUM = 5
RETRIES = 5
CORE_MODE = 0
REAL_MODE = 1
TARGETPROXIMITY = 20
TARGETFORCE = 1.0
UNKNOWN = -9999

class action_space():
    def __init__(self):
        self.x = [0, -5, 0, 5]
        self.y = [5, 0, -5, 0]
        
    def sample(self):
        import numpy as np
        i = np.random.randint(4)
        return (self.x[i], self.y[i])

    def maybe_right(self):
        import numpy as np
        i = np.random.randint(10)
        if i > 3:
            return (self.x[3], self.y[3])
        return (self.x[i], self.y[i])

    def action(self, i):
        return (self.x[i], self.y[i])

class Node(asyncio.DatagramProtocol, gym.Wrapper):
    def __init__(self, run_mode=REAL_MODE, configuration_file=None, log_file='logtest'):
        self.__nt_int = 'wlan0'
        self.__run_mode = run_mode
        self.__configuration_file = configuration_file
        self.__log_file = log_file
        self.__log_directory = f'{os.getcwd()}/logs'

        self.__loop = None

        # Node table
        self.__node_status = {} # node_id, node_state, node_position_x, node_position_y, node_last_communication_time, node_signal
        self.__node_signal = {} # 
        
        self.__initialization(self.__configuration_file)
        

        self.__loop = asyncio.get_event_loop()
        coro = self.__loop.create_datagram_endpoint(lambda: self, local_addr=('0.0.0.0', self.__port))
        asyncio.ensure_future(coro)
        #asyncio.ensure_future(self.__bioairing())
        

        if self.__run_mode == CORE_MODE :
            # Read configurate file
            pass
        elif self.__run_mode == REAL_MODE :
            import re, uuid
            self.__mac = ':'.join((re.findall('..', '%012x' % uuid.getnode())))
            print ("The MAC address in formatted and less complex way is : ", self.__mac) 

            self.__connection_with_drone()

            if self.__drone == None:
                raise ConnectionToDroneException()
            asyncio.ensure_future(self.__update_position())
    
    def reset(self):
        self.__initialization(self.__configuration_file)
        self.__thread = threading.Thread(target=self.run, args=(0,))
        self.__thread.daemon = True
        self.__thread.start()
        
    def step(self, action):
        vel_x = action[0]
        vel_y = action[1]

        for node_id, node_status in self.__node_status.items():
            node_signal = node_status['node_signal']
            print("SQ of", node_id,":",node_signal)
        # Get __position_x, __position_y
        self.__update_location(vel_x, vel_y)
        self.__update_reward()
        self.__update_done()

        return [self.__position_x, self.__position_y], self.__reward, self.__done, None

    def render(self):
        self.__update_core_position(self.__id, self.__state, self.__tentacle_state, self.__position_x, self.__position_y)

    def close(self):
        # thread close
        pass
    
    def __update_done(self):
        if self.__position_x < self.__range_x0:
            self.__done = True
            return
        if self.__position_x > self.__range_x1:
            self.__done = True
            return
        if self.__position_y < self.__range_y0:
            self.__done = True
            return
        if self.__position_y > self.__range_y1:
            self.__done = True
            return
        self.__done = False
        return
        
    def __update_reward(self):
        avg_signal = 0
        sig_count = 0 
        for node_id, node_status in self.__node_status.items():
            sq = node_status['node_signal']
            if sq != UNKNOWN:
                avg_signal += sq
                sig_count += 1
        if sig_count == 0 :
            avg_signal = UNKNOWN
        else :
            avg_signal /= sig_count
            
        if avg_signal == UNKNOWN:
            self.__reward = -10
        elif avg_signal > self.__highSQ:
            self.__reward = -1
        elif avg_signal >= self.__lowSQ:
            self.__reward = 1
        else:
            self.__reward = -1
       
    def __initialization(self, configuration_file):
        self.action_space = action_space()
        self.__reward = 0
        self.__done = False
        
        self.__distance = -1
        self.__range_x0 = 0
        self.__range_x1 = 1200
        self.__range_y0 = 0
        self.__range_y1 = 300
        
        self.__core_ip = '127.0.0.1'
        self.__drone = None
        self.__mac = None
        self.__id = -1 # Node Id
        self.__state = NodeState.Loading # Node State
        self.__position_x = None # X = longitude
        self.__position_y = None # Y = latitude
        self.__destinations = []
        self.__dest_index = 0
        self.__dest_id = None
        self.__dest_position_x = None
        self.__dest_position_y = None
        self.__origin_id = None
        self.__origin_position_x = None
        self.__origin_position_y = None
        self.__altitude = None 
        self.__tentacle_state = TentacleState.Forming
        self.__tentacle_id = UNKNOWN
        self.__tentacle_within_pos = UNKNOWN
        self.__port = 10800
        self.__incompltete_orphan = 0
        self.__hold = False
        
        # signal quality
        self.__max_groundspeed = 10
        self.__radio_range = 0
        self.__highSQ = 0.3
        self.__lowSQ = 0.1
        self.__equilibrium_zone = 0.1
        self.__equilibrium = 0
        self.__prev_vel_x = 0
        self.__prev_vel_y = 0
        
        config = configparser.ConfigParser()
        config.read(configuration_file)
        
        # core configuration
        if 'CORE_IP' in config['CORE']:
            self.__core_ip = config['CORE']['CORE_IP']

        if 'MAC' in config['CORE']:
            self.__mac = config['CORE']['MAC']

        if 'ID' in config['CORE']:
            self.__id = int(config['CORE']['ID'])
        
        if 'STATE' in config['CORE']:
            self.__state = config['CORE']['STATE']
            
            if self.__state == 'Origin': self.__state = NodeState.Origin
            elif self.__state == 'Destination': self.__state = NodeState.Destination
            elif self.__state == 'Free': self.__state = NodeState.Free
            elif self.__state == 'Orphan': self.__state = NodeState.Orphan

        if 'POSITION_X' in config['CORE']:
            self.__position_x = float(config['CORE']['POSITION_X'])

        if 'POSITION_Y' in config['CORE']:
            self.__position_y = float(config['CORE']['POSITION_Y'])

        if 'ALTITUDE' in config['CORE']:
            self.__altitude = float(config['CORE']['ALTITUDE'])

        if 'ORIGIN_ID' in config['CORE']:
            self.__origin_id = int(config['CORE']['ORIGIN_ID'])

            if 'ORIGIN_POSITION_X' in config['CORE']:
                self.__origin_position_x = float(config['CORE']['ORIGIN_POSITION_X'])
            else :
                print("No ORIGIN_POSITION_X")
                raise ValueError

            if 'ORIGIN_POSITION_Y' in config['CORE']:
                self.__origin_position_y = float(config['CORE']['ORIGIN_POSITION_Y'])
            else :
                print("No ORIGIN_POSITION_Y")
                raise ValueError
            
            self.__add_node_status(self.__origin_id, UNKNOWN, self.__origin_position_x, self.__origin_position_y, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN)
            self.__refresh_origin()

        if 'DEST_ID' in config['CORE']:
            self.__dest_id = int(config['CORE']['DEST_ID'])

            if 'DEST_POSITION_X' in config['CORE']:
                self.__dest_position_x = float(config['CORE']['DEST_POSITION_X'])
            else :
                print("No DEST_POSITION_X")
                raise ValueError
            

            if 'DEST_POSITION_Y' in config['CORE']:
                self.__dest_position_y = float(config['CORE']['DEST_POSITION_Y'])
            else :
                print("No DEST_POSITION_Y")
                raise ValueError
            
            self.__add_node_status(self.__dest_id, UNKNOWN, self.__dest_position_x, self.__dest_position_y, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN)
            self.__update_destination( self.__dest_id, 1)
            
        if 'PORT' in config['CORE']:
            self.__port = int(config['CORE']['PORT'])
            
        if 'RADIO_RANGE' in config['CORE']:
            self.__radio_range = float(config['CORE']['RADIO_RANGE'])
            
        if 'MAX_GROUNDSPEED' in config['CORE']:
            self.__max_groundspeed = float(config['CORE']['MAX_GROUNDSPEED'])

        # signal configuration
        if 'HIGH_SQ' in config['SIGNAL']:
            self.__highSQ = float(config['SIGNAL']['HIGH_SQ'])
        
        if 'LOW_SQ' in config['SIGNAL']:
            self.__lowSQ = float(config['SIGNAL']['LOW_SQ'])

        if 'EQUILIBRIUM_ZONE' in config['SIGNAL']:
            self.__equilibrium_zone = float(config['SIGNAL']['EQUILIBRIUM_ZONE'])

        if 'EQUILIBRIUM' in config['SIGNAL']:
            self.__equilibrium = float(config['SIGNAL']['EQUILIBRIUM'])

        # end of configuration

        print(f'Node {self.__id} is registered');

    def __update_destination(self, dest_id, update_type):
        done = False
        
        if update_type > 0:
            self.__destinations.append(dest_id)
        elif update_type < 0:
            if update_type == UNKNOWN:
                self.__dest_index = 0
            else :
                self.__dest_index += 1
                
            if self.__dest_index >= len(self.__destinations):
                self.__dest_index = len(self.__destinations) - 1
                done = True
                
            elif dest_id == UNKNOWN :
                self.__dest_index -= 1

        self.__change_tetacle_id(self.__destinations[self.__dest_index])
        self.__refresh_destination(self.__destinations[self.__dest_index])
        return done
    
    def __update_core_icon(self, node_id, icon):
        cmd = f'coresendmsg -a {self.__core_ip} NODE NUMBER={node_id} ICON={icon}'
        # start cmd
        # print(cmd)
        os.popen(cmd)
        
    def __update_location(self, vel_x, vel_y):
        
        expected_x, expected_y, wp_latitude, wp_longtitude = 0, 0, 0, 0
        wait_time = 0

        if (((self.__prev_vel_x < 0) and (vel_x >= 0)) or ((self.__prev_vel_x > 0) and (vel_x <= 0)) or ((self.__prev_vel_x == 0) and (vel_x != 0)) or ((self.__prev_vel_y < 0) and (vel_y >= 0)) or ((self.__prev_vel_y > 0) and (vel_y <= 0)) or ((self.__prev_vel_y == 0) and (vel_y != 0))) :
            wait_time = 0
        else:
            wait_time = 1.5 * ((vel_x**2) + (vel_y**2))

        vel_x = vel_x * self.__max_groundspeed
        vel_y = vel_y * self.__max_groundspeed

        if (vel_x >= self.__max_groundspeed):
            vel_y = (vel_y*self.__max_groundspeed)/vel_x
            vel_x = self.__max_groundspeed
        elif (vel_x <= -self.__max_groundspeed):
            vel_y = (vel_y*(0-self.__max_groundspeed))/vel_x
            vel_x = -self.__max_groundspeed

        if (vel_y >= self.__max_groundspeed):
            vel_x = (vel_x*self.__max_groundspeed)/vel_y
            vel_y = self.__max_groundspeed
        elif (vel_y <= -self.__max_groundspeed):
            vel_x = (vel_x*(0-self.__max_groundspeed))/vel_y
            vel_y = -self.__max_groundspeed
            
        if self.__run_mode == CORE_MODE:
            #await asyncio.sleep(0.5)
            time.sleep(0.5)
            self.__position_x = self.__position_x + (vel_x * 0.3)
            self.__position_y = self.__position_y + (vel_y * 0.3)
            #self.__update_core_position(self.__id, self.__state, self.__tentacle_state, self.__position_x, self.__position_y)

        elif self.__run_mode == REAL_MODE:
            self.__update_real_position(self.__position_x, self.__position_y, vel_x, vel_y)
            pass
        elif self.__run_mode == 2:
            pass
        

    def __update_core_position(self, node_id, state, tentacle_state, position_x, position_y):
        cmd = f'coresendmsg -a {self.__core_ip} NODE NUMBER={node_id} NAME={node_id}_{state}_{tentacle_state} X_POSITION={int(position_x)} Y_POSITION={int(position_y)}'
        # start cmd
        # print(cmd)
        os.popen(cmd)

    def __update_real_position(self, position_x, position_y, vel_x, vel_y):
        print(f'From ({position_x}, {position_y}) move with vel_x : {vel_x}, vel_y : {vel_y}')
        
        self.__drone.offboard.set_velocity_ned(vel_y, vel_x, 0.0, 0.0)

    def __get_destination_id(self) :
        return self.__dest_id

    def __change_state(self, state):
        self.__state = state

    def __change_tetacle_id(self, tentacle_id):
        self.__tentacle_id = tentacle_id
    
    def __change_tentacle_within_pos(self, tentacle_within_pos):
        self.__tentacle_within_pos = tentacle_within_pos
        
    def __change_tentacle_state(self, tentacle_state):
        self.__tentacle_state = tentacle_state
    
    def __refresh_destination(self, dest_id):
        self.__dest_position_x = self.__node_status[dest_id]['node_position_x']
        self.__dest_position_y = self.__node_status[dest_id]['node_position_y']

    def __refresh_origin(self):
        self.__origin_position_x = self.__node_status[self.__origin_id]['node_position_x']
        self.__origin_position_y = self.__node_status[self.__origin_id]['node_position_y']

    # About communication
    # Called when a connection is made.
    def connection_made(self, transport: asyncio.DatagramTransport):
        self.__transport = transport
        sock = self.__transport.get_extra_info('socket')
        sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        self.__broadcast()

    # Called when a datagram is received.
    def datagram_received(self, data, addr: Address):
        data = json.loads(data.decode())
        node_mac = data.get('node_mac')

        if node_mac == self.__mac :
            # print("-- From myself")
            return

        if self.__state == NodeState.Loading :
            print("-- Drone is not ready")
            return

        node_id = data.get('node_id')
        node_state = data.get('node_state')
        node_state = NodeState(node_state)
        node_position_x = data.get('node_position_x')
        node_position_y = data.get('node_position_y')
        tentacle_id = data.get('tentacle_id')
        tentacle_state = data.get('tentacle_state')
        tentacle_state = TentacleState(tentacle_state)
        tentacle_within_pos = data.get('tentacle_within_pos')
        
        node_signal = self.__get_signal_quality(node_mac, node_position_x, node_position_y)

        log = f"-- From : {node_mac} / SQ : {node_signal} / node_id : {node_id}, node_state : {node_state}, node_position_x : {node_position_x}, node_position_y : {node_position_y}, tentacle_id : {tentacle_id}, tentacle_state : {tentacle_state} tentacle_within_pos : {tentacle_within_pos}"
        
        #print(log)

        # write data to log.file
        timestr = time.strftime("%Y%m%d-%H%M%S")
        try : 
            if not (os.path.isdir(self.__log_directory)):
                os.makedirs(os.path.join(self.__log_directory))
        except OSError as e:
            if e.errno != errno.EEXIST:
                print("Failed to create directory !")
                raise
        
        with open(f'{os.getcwd()}/logs/{self.__id}_{self.__log_file}.log', 'a') as f:
            log = f'{timestr} ID {self.__id} X {self.__position_x} Y {self.__position_y} STATE {self.__state} \n' + log + '\n'
            f.write(log)
        
        self.__add_node_status(node_id, node_state, node_position_x, node_position_y, node_signal, tentacle_id, tentacle_state, tentacle_within_pos)
        self.__update_status(node_id, node_state, node_signal, tentacle_id, tentacle_state, tentacle_within_pos)
        
    def __broadcast(self):
        connectionInfo = ('<broadcast>', self.__port)

        data = json.dumps({
                'node_id': self.__id,
                'node_state': self.__state.value,
                'node_position_x': self.__position_x,
                'node_position_y': self.__position_y,
                'node_mac' : self.__mac,
                'tentacle_id' : self.__tentacle_id,
                'tentacle_state' : self.__tentacle_state.value,
                'tentacle_within_pos' : self.__tentacle_within_pos,
        })

        self.__transport.sendto(data.encode(), connectionInfo)
        
        # print(f"-- Broadcast : {self.__mac} / node_id : {self.__id}, node_state : {self.__state}, node_position_x : {self.__position_x}, node_position_y : {self.__position_y}, tentacle_id : {self.__tentacle_id}, tentacle_within_pos : {self.__tentacle_within_pos}')

        self.__loop.call_later(1, self.__broadcast)

    def __get_signal_quality(self, node_mac, node_position_x, node_position_y):
        #print("position:", self.__position_x, self.__position_y, node_position_x, node_position_y)
        if self.__run_mode == CORE_MODE : # CORE
            rxSignal = math.sqrt((self.__position_x - node_position_x)**2+(self.__position_y - node_position_y)**2)
            #print("SQ:", rxSignal)
            # current value in ini is 330
            if (rxSignal < self.__radio_range) :
                # rxSignal = 50 - (10 * math.log(rxSignal/4))
                rxSignal = 1 - (rxSignal / self.__radio_range)
            else :
                rxSignal = UNKNOWN
            #print("rxSignal:", rxSignal)

        elif self.__run_mode == REAL_MODE : # Real
            check_signal_cmd = f'iw dev {self.__nt_int} station get {node_mac} | grep signal: | grep -oE ([-]{{1}}[0-9]*){{1}}'
            result = os.popen(check_signal_cmd).read()

            if result == '':
                rxSignal = UNKNOWN
            
            else :
                rxSignal = int(result.strip().split('\n')[0])
                rxSignal = 1.0 + (rxSignal/101)
        
        return rxSignal


    # Called by __init__ and datagram_received
    def __add_node_status(self, node_id, node_state, node_position_x, node_position_y, node_signal, tentacle_id, tentacle_state, tentacle_within_pos):
        # add or update nodes statue
        if node_id in self.__node_status:
            # update
            self.__node_signal[node_id].pop(0)
            self.__node_signal[node_id].append(node_signal)

            avg_signal = 0
            sig_count = 0 

            for i in self.__node_signal[node_id] :
                if i != UNKNOWN:
                    avg_signal += i
                    sig_count += 1
            
            if sig_count == 0 :
                avg_signal = UNKNOWN
            else :
                avg_signal /= sig_count

            self.__node_status[node_id]['node_state'] = node_state
            self.__node_status[node_id]['node_position_x'] = node_position_x
            self.__node_status[node_id]['node_position_y'] = node_position_y
            self.__node_status[node_id]['node_last_connection_time'] = 0
            self.__node_status[node_id]['node_signal'] = avg_signal
            self.__node_status[node_id]['tentacle_id'] = tentacle_id
            self.__node_status[node_id]['tentacle_state'] = tentacle_state
            self.__node_status[node_id]['tentacle_within_pos'] = tentacle_within_pos

        else :
            # add
            self.__node_status[node_id] = {}
            self.__node_signal[node_id] = [node_signal] * SAMPLENUM

            self.__node_status[node_id]['node_state'] = node_state
            self.__node_status[node_id]['node_position_x'] = node_position_x
            self.__node_status[node_id]['node_position_y'] = node_position_y
            self.__node_status[node_id]['node_last_connection_time'] = 0
            self.__node_status[node_id]['node_signal'] = node_signal
            self.__node_status[node_id]['tentacle_id'] = tentacle_id
            self.__node_status[node_id]['tentacle_state'] = tentacle_state
            self.__node_status[node_id]['tentacle_within_pos'] = tentacle_within_pos

        #print("node id of this node is",self.__id)
        #print("SQ of", node_id,":",self.__node_status[node_id]['node_signal'])
        
        
    def __update_status(self, node_id, node_state, node_signal, tentacle_id, tentacle_state, tentacle_within_pos):
        
        # if node_id == self.__origin_id :
        #     self.__refresh_origin()
        # else :
        #     self.__update_destination(UNKNOWN, 0)
        
        if node_signal != UNKNOWN :
            if self.__tentacle_id == tentacle_id :
                if tentacle_state is TentacleState.Next_Destination and (self.__state is NodeState.Orphan or self.__state is NodeState.Free):
                    self.__change_tentacle_state(TentacleState.Forming)
                    self.__update_destination(self.__get_destination_id(), -1)
                    pass
                elif tentacle_state is TentacleState.Complete and self.__state is NodeState.Free:
                    self.__change_tentacle_state(tentacle_state)
                    self.__change_state(NodeState.Extra)
                    self.__update_destination(self.__origin_id, 1)
                    self.__update_destination(self.__origin_id, UNKNOWN)                    
                    pass
                elif tentacle_state is TentacleState.Damaged:
                    self.__change_tentacle_state(tentacle_state)
                    
                    if self.__state is NodeState.Tip or self.__state is NodeState.Backbone:
                        self.__hold = True
                elif self.__tentacle_state is TentacleState.Complete and tentacle_state is TentacleState.Forming:
                    pass
                else:
                    self.__change_tentacle_state(tentacle_state)
                    
                if self.__tentacle_state is not TentacleState.Damaged and self.__state is NodeState.Tip and (node_state is NodeState.Tip or node_state is NodeState.Backbone) and self.__tentacle_within_pos < tentacle_within_pos :
                    self.__change_state(NodeState.Backbone)
                    print(f'Node {self.__id} changed from a tip to backbone')
            
            if node_state is NodeState.Destination and self.__tentacle_id is node_id:
                if self.__tentacle_state is TentacleState.Forming and self.__state is NodeState.Tip:
                    self.__change_tentacle_state(TentacleState.Complete)
                    self.__change_state(NodeState.Backbone)
                    print(f'Node {self.__id} changed from a tip to backbone')
                    print(f'Tentacle {self.__tentacle_id} changed from forming to complete')
                    
    # About drone
    async def __connection_with_drone(self):
        self.__drone = System()
        await self.__drone.connect(system_address="udp://:14551")

        print("Waiting for drone...")
        
        async for state in self.__drone.core.connection_state():
            if state.is_connected:
                mav_sys_id = await self.__drone.param.get_int_param('MAV_SYS_ID')
                print(f"Drone discovered with MAV_SYS_ID: {mav_sys_id}")
                self.__droneID = mav_sys_id # MAV_SYS_ID 를 사용하자
                break

    async def __update_position(self):
        async for position in self.__drone.telemetry.position():
            # print(f"cur_position: {position}")
            self.__position_x = position.longitude_deg
            self.__position_y = position.latitude_deg
            self.__altitude = position.altitude_m

    async def __change_mode_to_offboard(self):
        print("-- Arming")
        await self.__drone.action.arm()

        print("-- Setting initial setpoint")
        await self.__drone.offboard.set_velocity_ned(VelocityNEDYaw(0.0, 0.0, 0.0, 0.0))


        print("-- Starting offboard mode")
        try:
            await self.__drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Landing")
            await self.__drone.action.returnToLaunch()

    async def __takeoff_m(self, height):
        print(f"-- Go up to {height}m")
        await self.__drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1.0, 0.0))
        
        async for position in self.__drone.telemetry.position():
            if abs(position.altitude_m - height) < 0.5 :
                print(f'Current Altitude : {position.altitude_m} -> Stop')
                break

        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    async def __bioairing(self):
        while True:
            if self.__state == NodeState.Loading:
                print("-- Drone is not ready")
                await asyncio.sleep(5)
            else :
                if self.__state is NodeState.Origin or self.__state is NodeState.Destination :
                    self.__update_core_position(self.__id, self.__state, self.__tentacle_state, self.__position_x, self.__position_y)
                    break

                vel_x, vel_y = 0, 0
                o_x, o_y = 0, 0
                add_x, add_y = 0, 0
                factor = 0
                adjOrigin, adjDest, adjTip, adjBackbone, adjFree = 0, 0, 0, 0, 0
                next_attractor = -1
                
                vel_x, vel_y = self.__virtual_force_to_destination()
                virtx, virty = vel_x, vel_y
                
                # Get position of the further node
                next_attractor = self.__get_further_node_on_same_tentacle()

                for node_id, node_status in self.__node_status.items():
                    node_state = node_status['node_state']
                    node_position_x = node_status['node_position_x']
                    node_position_y = node_status['node_position_y']
                    node_signal = node_status['node_signal']
                    tentacle_id = node_status['tentacle_id']
                    tentacle_within_pos = node_status['tentacle_within_pos'] 
                    
                    if node_signal == UNKNOWN:
                        continue

                    add_x, add_y = self.__get_next_velocities(node_state, node_signal, node_position_x, node_position_y)
                    
                    if node_state is NodeState.Destination:
                        if (node_id == self.__get_destination_id()):
                            vel_x += add_x
                            vel_y += add_y
                            vel_x -= virtx
                            vel_y -= virty
                            adjDest += 1
                            
                            if self.__state is NodeState.Free:
                                pass
                            
                            elif self.__state is NodeState.Extra:
                                self.__update_destination(node_id, -1)
                        
                    elif node_state is NodeState.Origin:
                        adjOrigin += 1
                        
                        if next_attractor < 0 or self.__tentacle_within_pos == 0:    
                            vel_x += add_x
                            vel_y += add_y
                            
                        if self.__state is NodeState.Tip or self.__state is NodeState.Backbone:
                            o_x += add_x
                            o_y += add_y
                        
                        elif self.__state is NodeState.Orphan:
                            self.__change_state(NodeState.Free)
                        
                        if self.__state is NodeState.Free:
                            adjTip += 1
                        
                    elif node_state is NodeState.Tip:
                        adjTip += 1

                        if self.__state != NodeState.Tip and self.__state != NodeState.Backbone:
                            vel_x += add_x
                            vel_y += add_y
                            
                    elif node_state is NodeState.Backbone:
                        if self.__tentacle_id >= tentacle_id :
                            adjBackbone += 1
                            
                        condi = (self.__state is NodeState.Tip or self.__state is NodeState.Backbone)
                            
                        if condi and self.__tentacle_within_pos > tentacle_within_pos :
                            o_x += add_x
                            o_y += add_y
                        
                        if (condi and self.__tentacle_within_pos == tentacle_within_pos + 1) or (self.__state != NodeState.Backbone and tentacle_id <= self.__tentacle_id and tentacle_within_pos == next_attractor) or self.__state is NodeState.Reinforce:
                            vel_x += add_x
                            vel_y += add_y

                    elif (node_state is NodeState.Extra or node_state is NodeState.Reinforce) and (self.__state is NodeState.Extra or self.__state is NodeState.Reinforce) :
                        vel_x += add_x
                        vel_y += add_y
                        
                    elif node_state is NodeState.Free and self.__state is NodeState.Free:
                        vel_x += add_x
                        vel_y += add_y
                        
                if math.sqrt(vel_x**2 + vel_y**2) < self.__equilibrium_zone:
                    self.__equilibrium += 1
                    # pass
                else :
                    self.__equilibrium = 0
                    
                near_node_count = (adjBackbone + adjDest + adjTip + adjOrigin)

                if self.__state is NodeState.Orphan and (near_node_count > 0):
                    self.__change_state(NodeState.Free)

                elif self.__state is NodeState.Orphan and (near_node_count == 0):
                    vel_x, vel_y = self.__virtual_force_to_origin()

                elif self.__state is NodeState.Tip or self.__state is NodeState.Backbone :
                    if near_node_count > 0 :
                        self.__incompltete_orphan = 0
                        
                    if self.__hold :
                        vel_x, vel_y = 0, 0
                        
                    elif self.__state is NodeState.Tip and near_node_count < 1:
                        #change to Orphan
                        # damaged
                        self.__incompltete_orphan += 1
                        
                        if self.__incompltete_orphan > (len(self.__node_status) + 4) * RETRIES :
                            self.__change_state(NodeState.Orphan)                            
                            vel_x, vel_y = self.__virtual_force_to_origin()
                            
                            print(f'Tip Node {self.__id} changed to Orphan')
                    elif self.__state is NodeState.Backbone and near_node_count < 2 :
                        self.__incompltete_orphan += 1
                        
                        if self.__incompltete_orphan > (len(self.__node_status) + 4) * RETRIES :
                            vel_x, vel_y = 0, 0
                            self.__hold = True
                            self.__change_state(NodeState.Tip)
                            self.__incompltete_orphan = 0
                    elif (self.__state is NodeState.Tip or self.__state is NodeState.Backbone) and (o_x > 0 or o_y >0) and math.sqrt(o_x**2 + o_y**2) < self.__equilibrium_zone/5 :
                        vel_x, vel_y = self.__virtual_force_to_origin()
                        
                        print(f'Node {self.__id} moving towards origin due to a weak attraction')
                    elif (self.__equilibrium > 0) :
                        vel_x, vel_y = 0, 0

                elif near_node_count == 0:
                    # Change to Orphan
                    self.__incompltete_orphan += 1
                    
                    if self.__incompltete_orphan > (len(self.__node_status) + 1) * RETRIES :
                        self.__change_state(NodeState.Orphan)
                        
                        vel_x, vel_y = self.__virtual_force_to_origin()
                        
                        print(f'Orphan Node {self.__id} moving to origin at {self.__origin_position_x}, {self.__origin_position_y}')

                elif self.__state is NodeState.Free and ((adjTip + adjBackbone) == 1) and self.__equilibrium > RETRIES/2 :
                    self.__change_state(NodeState.Tip)
                    self.__change_tentacle_within_pos(next_attractor + 1)
                    print("New tip node added")

                elif self.__state is NodeState.Extra :
                    vel_x, vel_y = self.__virtual_force_to_destination()
                
                if ((self.__equilibrium_zone >= 1.0)):
                    vel_x = 0
                    vel_y = 0
                
                # update location
                await self.__update_location(vel_x, vel_y)
                self.__prev_vel_x = vel_x
                self.__prev_vel_y = vel_y
                await asyncio.sleep(0.5)

    def __virtual_force_to_destination(self) :
        vel_x, vel_y = 0, 0
        if abs(self.__dest_position_x - self.__position_x) < TARGETPROXIMITY :
            vel_x = (self.__dest_position_x - self.__position_x)/(TARGETPROXIMITY*1.0)
        elif (self.__dest_position_x > self.__position_x) :
            vel_x = TARGETFORCE
        else :
            vel_x = 0 - TARGETFORCE

        if (abs(self.__dest_position_y - self.__position_y) < TARGETPROXIMITY) :
            vel_y = (self.__dest_position_y - self.__position_y)/(TARGETPROXIMITY*1.0)
        elif (self.__dest_position_y > self.__position_y) :
            vel_y = TARGETFORCE
        else : 
            vel_y = 0 - TARGETFORCE

        ratio = abs(self.__dest_position_y - self.__position_y)/(abs(self.__dest_position_x - self.__position_x)*1.0)
        if (ratio < 1.0) :
            vel_y = vel_y * ratio
        
        ratio = abs(self.__dest_position_x - self.__position_x)/(abs(self.__dest_position_y - self.__position_y)*1.0)
        if (ratio < 1.0) :
            vel_x = vel_x * ratio
        
        return vel_x, vel_y
    
    def __virtual_force_to_origin(self) :
        vel_x, vel_y = 0, 0
        
        if abs(self.__origin_position_x - self.__position_x) < TARGETPROXIMITY :
            vel_x = 0
        elif (self.__origin_position_x > self.__position_x) :
            vel_x = TARGETFORCE
        else :
            vel_x = 0 - TARGETFORCE

        if (abs(self.__origin_position_y - self.__position_y) < TARGETPROXIMITY) :
            vel_y = 0
        elif (self.__origin_position_y > self.__position_y) :
            vel_y = TARGETFORCE
        else : 
            vel_y = 0 - TARGETFORCE

        ratio = abs(self.__origin_position_y - self.__position_y)/(abs(self.__origin_position_x - self.__position_x)*1.0)
        if (ratio < 1.0) :
            vel_y = vel_y * ratio
        
        ratio = abs(self.__origin_position_x - self.__position_x)/(abs(self.__origin_position_y - self.__position_y)*1.0)
        if (ratio < 1.0) :
            vel_x = vel_x * ratio
        
        return vel_x, vel_y
    
    def __get_further_node_on_same_tentacle(self) :
        next_attractor = -1
        
        for node_id, node_status in self.__node_status.items():
            node_state = node_status['node_state']
            node_position_x = node_status['node_position_x']
            node_position_y = node_status['node_position_y']
            node_signal = node_status['node_signal']
            tentacle_id = node_status['tentacle_id']
            tentacle_within_pos = node_status['tentacle_within_pos'] 
            
            if node_signal != UNKNOWN :
                if (node_state is NodeState.Tip or (node_state is NodeState.Backbone and self.__tentacle_id > tentacle_id)) and (tentacle_within_pos >= next_attractor) :
                    next_attractor = tentacle_within_pos
                
                elif node_state is NodeState.Backbone and (self.__state is NodeState.Extra and tentacle_within_pos >= next_attractor):
                    next_attractor = tentacle_within_pos
        
        return next_attractor
    
    def __get_next_velocities(self, node_state, node_signal, node_position_x, node_position_y) :
        add_x, add_y = 0, 0
        factor = 0
        
        if node_signal > self.__highSQ :
            factor = 1 * (node_signal - self.__highSQ) / (1 - self.__highSQ)
        elif node_signal < self.__lowSQ :
            factor = -self.__lowSQ / node_signal
        else :
            factor = 0

        if node_state == NodeState.Destination and factor > 0 :
            factor = 0
        elif node_state is NodeState.Tip or node_state is NodeState.Origin or node_state is NodeState.Backbone or node_state is NodeState.Destination :
            pass
        elif self.__state is NodeState.Tip or self.__state is NodeState.Backbone:
            factor = 0
        elif factor < 0 :
            factor = 0

        add_x = factor * (self.__position_x - node_position_x) / math.sqrt((self.__position_x - node_position_x)**2 + (self.__position_y - node_position_y)**2)
        add_y = factor * (self.__position_y - node_position_y) / math.sqrt((self.__position_x - node_position_x)**2 + (self.__position_y - node_position_y)**2)
        
        return add_x, add_y
    
    # Set the drone be ready
    def run(self, height):
        # Do
        print("Start BioAIR")
        
        if self.__run_mode == REAL_MODE :
            self.__change_mode_to_offboard()
            self.__takeoff_m(height)

        print("Drone is ready")
        self.__loop.run_forever()


