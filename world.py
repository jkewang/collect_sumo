#in this pyfile, a world is generated and run autonoumously to generate basic policy
import os,sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],"tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable SUMO_HOME")

config_path = "/home/jkwang/learn_sumo/quickstart/quickstart.sumo.cfg"
sumoBinary = "/usr/bin/sumo"
sumoguiBinary = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary,"-c",config_path,"--collision.action","remove","--start","--no-step-log","--no-warnings","--no-duration-log"]

import traci
import traci.constants as tc
import math
import numpy as np

class world(object):
    def __init__(self,f):
        traci.start(sumoCmd)
        self.f = f
        self.step_num = 0
        self.VehicleIds = []
        self.OccMapState = np.zeros((20,7))
        self.VehicleState = [0,0,0]
        self.RoadState = [0 for i in range(9)]

        self.roadMaxLane = {
            "-gneE8": 2,
            "gneE8": 2,
            "-gneE9": 2,
            "gneE9": 2,
            "-gneE10": 2,
            "gneE10": 2,
            "-gneE11": 2,
            "gneE11": 2,
            "-gneE0": 1,
            "gneE0": 1,
            "-gneE1": 1,
            "gneE1": 1,
            "-gneE2": 1,
            "gneE2": 1,
            "-gneE3": 1,
            "gneE3": 1,
            "-gneE4": 1,
            "gneE4": 1,
            "-gneE5": 1,
            "gneE5": 1,
            "-gneE6": 1,
            "gneE6": 1,
            "-gneE7": 1,
            "gneE7": 1,
        }
        self.roadMinLane = 0

        self.trafficPos_mapping = {
            "cross_3": [-1000, 0],
            "cross_4": [0, 0],
            "cross_5": [1000, 0],
            "cross_2": [1000, 1000],
            "cross_1": [0, 1000],
            "cross_0": [-1000,1000],
            "cross_6": [-1000,-1000],
            "cross_7": [0,-1000],
            "cross_8": [1000,-1000]
        }

    def reset(self):
        traci.load(["-c",config_path,"--collision.action","remove","--no-step-log","--no-warnings","--no-duration-log"])
        print("reseting")
        traci.simulationStep()
        self.VehicleIds = traci.vehicle.getIDList()
        for vehId in self.VehicleIds:
            traci.vehicle.subscribe(vehId,(tc.VAR_SPEED,tc.VAR_LANE_INDEX,tc.VAR_DISTANCE,tc.VAR_ANGLE))


    def step(self):
        traci.simulationStep()
        self.VehicleIds = traci.vehicle.getIDList()
        for subvehId in self.VehicleIds:
            traci.vehicle.subscribe(subvehId,(tc.VAR_SPEED,tc.VAR_POSITION,tc.VAR_LANE_INDEX,tc.VAR_DISTANCE,tc.VAR_ANGLE))

        for vehId in self.VehicleIds:
            occ_state,VehicleState,RoadState = self.perception(vehId)
            self.f.write(str(occ_state)+"\n")
            self.f.write(str(VehicleState)+"\n")
            self.f.write(str(RoadState)+"\n")

    def perception(self,vehId):
        #-------------------to get the vehicle state-----------------------
        VehicleParam = traci.vehicle.getSubscriptionResults(vehId)
        vehicleSpeed = VehicleParam[tc.VAR_SPEED]
        vehicleAngle = (VehicleParam[tc.VAR_ANGLE]/180)*math.pi
        vehicleX = VehicleParam[tc.VAR_POSITION][0]
        vehicleY = VehicleParam[tc.VAR_POSITION][1]

        self.VehicleState = [vehicleSpeed,math.cos(vehicleAngle),math.sin(vehicleAngle)]

        #--------------------to calculate the ocupanied state---------------
        AllVehicleParams = []
        for subvehicleId in self.VehicleIds:
            subvehicleParam =traci.vehicle.getSubscriptionResults(subvehicleId)
            if subvehicleId != vehId:
                AllVehicleParams.append(subvehicleParam)
        LOW_X_BOUND = -6
        HIGH_X_BOUND = 6
        LOW_Y_BOUND = -10
        HIGH_Y_BOUND = 30
        self.OccMapState = np.zeros((20, 7))
        for VehicleParam in AllVehicleParams:
            VehiclePos = VehicleParam[tc.VAR_POSITION]
            rol = math.sqrt((VehiclePos[0]-vehicleX)**2+(VehiclePos[1]-vehicleY)**2)
            theta = math.atan2(VehiclePos[1]-vehicleY,VehiclePos[0]-vehicleX)
            reltheta = theta + vehicleAngle
            relX = rol*math.cos(reltheta)
            relY = rol*math.sin(reltheta)
            if (relX>LOW_X_BOUND and relX<HIGH_X_BOUND) and (relY>LOW_Y_BOUND and relY<HIGH_Y_BOUND):
                indexX = int((6 + relX)/2 - 0.5)
                indexY = int((30 - relY)/2 - 0.5)
                self.OccMapState[indexY,indexX] = 1.0

        self.OccMapState = self.OccMapState.reshape(-1)

        #----------------------to get the RoadState----------------------------
        #RoadState: [leftcan rightcan distance r y g leftava centerava rightava]

        self.RoadState = [1.0 for i in range(9)]
        now_laneindex = traci.vehicle.getSubscriptionResults(vehId)[tc.VAR_LANE_INDEX]
        now_roadid = traci.vehicle.getRoadID(vehId)
        print(now_roadid)
        if now_roadid[0] != ':':
            maxLaneNumber = self.roadMaxLane[now_roadid]
            minLaneNumber = self.roadMinLane
            if now_laneindex +1 > maxLaneNumber:
                self.RoadState[0] = 0
            elif now_laneindex -1 < minLaneNumber:
                self.RoadState[1] = 0

            nextTlsId = traci.vehicle.getNextTLS(vehId)
            if nextTlsId != []:
                #print(traci.trafficlight.getControlledLinks(nextTlsId[0][0]))
                print(traci.trafficlights.getRedYellowGreenState("cross_4"))
                print(traci.trafficlights.getControlledLinks("cross_4"))

            lane = traci.vehicle.getLaneID(vehId)
            TL = traci.trafficlights.getIDList()
            print(lane)
            for i in TL:
                links = traci.trafficlights.getControlledLinks(i)
                #print("links0,0,0:",links[0][0][0])
                index = 0
                for link in links:
                    if (link[0][0] == lane):
                        phases = traci.trafficlights.getRedYellowGreenState(i)
                        phase = phases[index]
                        TLS_pos_x, TLS_pos_y = self.trafficPos_mapping[i][0],self.trafficPos_mapping[i][1]
                        distance = math.sqrt((vehicleX-TLS_pos_x)**2+(vehicleY-TLS_pos_y)**2)/1000

                        self.f.write("Vehicle " + str(vehId) + " is on the lane " + str(lane) + "nextTLS " + i + " with phase " + phase + " distance:" + str(distance) + "\n")
                        self.RoadState[2] = distance
                        if phase == 'g' or phase == 'G':
                            self.RoadState[3] = 0
                            self.RoadState[4] = 0
                            self.RoadState[5] = 1
                        elif phase == 'y' or phase == 'Y':
                            self.RoadState[3] = 0
                            self.RoadState[4] = 1
                            self.RoadState[5] = 0
                        else:
                            self.RoadState[3] = 1
                            self.RoadState[4] = 0
                            self.RoadState[5] = 0
                        break
                    index += 1

        return [self.OccMapState,self.VehicleState,self.RoadState]

f = open("data.txt",'w')
myworld = world(f)
i=0
while(i<200):
    myworld.step()
    i+=1
f.close()
