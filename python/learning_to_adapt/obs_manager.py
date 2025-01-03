import numpy as np

class ObsManager:
    def __init__(self):
        # initialise obs dict
        self.obsDict = {
            # base state obs
            "baseRot": [3,0], "baseAngVel": [3,0], "baseLinVel": [3,0], "baseHeight": [1,0],
            # joint state obs
            "jointPos": [12,0], "jointVel": [12,0], "jointTrq": [12,0],
            # feet contact states obs
            "feetContact": [4,0],
            # velocity commands obs
            "velCmd": [3,0], "accCmd": [3,0],
            # feet reference commands obs
            "desFeetContact": [4,0], "refFootX": [4,0], "refFootY": [4,0], "refFootZ": [4,0],
            # phase variable obs
            "phaseVariable": [4,0],
            # froude number obs
            "frVelCmd": [3,0], "frGaitStability": [1,0],
            # bio metrics obs
            "trqSaturation": [12,0],
            # gait selection module action
            "gsAction": [1,0],
            # gait transition flag
            "transFlag": [1,0]
        }
        self.obsNames = list(self.obsDict.keys())
        self.obsLengths = []
        for i in range(len(self.obsNames)):
            self.obsLengths.append(self.obsDict[self.obsNames[i]][0])
        
    def updateAllObs(self, sim, scheduler, velCmd, trqLims, robotID=0):
        # update base state
        self.obsDict["baseRot"][1] = sim.get_base_rotation_matrix(robotID)[2,:]
        self.obsDict["baseAngVel"][1] = sim.get_base_angular_velocity(robotID)
        self.obsDict["baseLinVel"][1] = sim.get_base_linear_velocity(robotID)
        self.obsDict["baseHeight"][1] = sim.get_base_position(robotID)[2]
        # update joint states
        self.obsDict["jointPos"][1] = sim.get_joint_positions(robotID)
        self.obsDict["jointVel"][1] = sim.get_joint_velocities(robotID)
        self.obsDict["jointTrq"][1] = sim.get_joint_efforts(robotID)
        # update feet contact states
        self.obsDict["feetContact"][1] = sim.get_limb_contact_states(robotID)
        # update velocity command
        self.obsDict["velCmd"][1] = velCmd
        # update feet reference commands
        self.obsDict["desFeetContact"][1] = scheduler.gaitData().contact_state_scheduled
        self.obsDict["refFootX"][1] = scheduler.gaitData().ref_foot_x
        self.obsDict["refFootY"][1] = scheduler.gaitData().ref_foot_y
        self.obsDict["refFootZ"][1] = scheduler.gaitData().ref_foot_z
        # update phase variable
        self.obsDict["phaseVariable"][1] = scheduler.gaitData().phase_variable
        # update froude number obs
        self.obsDict["frVelCmd"][1] = scheduler.gaitData().froudeCmd_
        self.obsDict["frGaitStability"][1] = scheduler.gaitData().FrStab
        # update bio metrics
        self.obsDict["trqSaturation"][1] = trqLims
        # update trans flag
        self.obsDict["transFlag"][1] = scheduler.gaitData().trans_flag
        
    def updateSpecificOb(self, obLabel, obValue):
        self.obsDict[obLabel][1] = obValue
        
    def generateObsArray(self, obsStructure):
        indx = 0
        obsLength = 0
        obsTotalLength = 0
        for i in range(len(obsStructure)):
            obsIndx = self.obsNames.index(obsStructure[i])
            obsLength += self.obsLengths[obsIndx]
        obs = np.zeros(obsLength)
        for i in range(len(obsStructure)):
            obsIndx = self.obsNames.index(obsStructure[i])
            obsLength = self.obsLengths[obsIndx]
            obs[indx:indx+obsLength] = self.obsDict[obsStructure[i]][1]
            indx += obsLength
        return obs


