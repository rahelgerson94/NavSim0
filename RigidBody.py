import numpy as np
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, matmul, array
#import debug as db
X = 0
Y = 1
Z = 2
class RigidBody:
    def __init__(self,  
                 rInI, 
                 vInB, 
                 aInB,
                 rotationSequence = [0,0,0],
                 dt = 1/100,):
        self.updateIB(rotationSequence)
        self.aInB = aInB
        self.vInB = vInB
        self.rInB = matmul(self.IB.T, rInI)
        
        self.rInI = rInI
        self.vInI =  matmul(self.IB, self.vInB)
        self.aInI =  matmul(self.IB, self.aInB)
        self.rInIhist = []
        self.vInIhist = []
        self.aInIhist = []

        self.thetaXrad = 0
        self.thetaYrad = 0
        self.thetaZrad = 0
        self.dt = dt
        
    def update(self, rotationSeqeunce, aInB):
         
        self.updateIB(rotationSeqeunce) #TODO: un-hard code this for target orientation that varies with time. 
        
         
        self.aInI = matmul(self.IB, aInB)
        self.vInI = self.vInI + self.aInI*self.dt
        self.rInI = self.rInI + self.vInI*self.dt
    def integrate(self, fx):
        pass
    def storeStates(self ):
        self.aInIhist.append(self.aInI)
        self.vInIhist.append(self.vInI)
        self.rInIhist.append(self.rInI)
    def toInertial(self, aInB, vInB):
        self.aInI = matmul(self.IB, aInB)
        self.vInI = matmul(self.IB, vInB)
    def printStates(self):
        print(f"rInI: {self.rInI}")
        print(f"vInI: {self.vInI}")
        print(f"aInI: {self.aInI}")
        print()
        print(f"---- [IB] ---")
        for row in self.IB:
            print("  ".join(f"{elem:.2f}" for elem in row))

    def updateIB(self, rotationSequence):
        """
        
        Compute the transformation from inertial to body frame [BN]
        via the Euler 321 rotation sequence.

        1st rot is about the 3rd body axis, zb
        2nd rot is about the 2nd body axis, yb
        3rd rot is about the 1st body axis, xb
        
        Parameters:
            rotationSequence[0]: angle of rot, about body z-axis in deg
            rotationSequence[2]: angle of rot, about body y-axis in deg
            rotationSequence[3]: angle of rot, about body x-axis in deg

        Returns:
            dcm: 3x3 numpy array representing the DCM
        """
        thetaZ = deg2rad(rotationSequence[0])
        thetaY = deg2rad(rotationSequence[1])
        thetaX = deg2rad(rotationSequence[2]) 
        if not(thetaY >= -90 and thetaY <= 90):
            
            print("WARNING! encontered signularity in θy")

        # Rotation matrix about x-axis
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(thetaX), -np.sin(thetaX)],
            [0, np.sin(thetaX), np.cos(thetaX)]
        ])

        # Rotation matrix about y-axis
        R_y = np.array([
            [np.cos(thetaY), 0, np.sin(thetaY)],
            [0, 1, 0],
            [-np.sin(thetaY), 0, np.cos(thetaY)]
        ])

        # Rotation matrix about z-axis
        R_z = np.array([
            [np.cos(thetaZ), -np.sin(thetaZ), 0],
            [np.sin(thetaZ), np.cos(thetaZ), 0],
            [0, 0, 1]
        ])

        # The 321 rotation sequence is Z (thetaZ), Y (thetaY), X (thetaX)
        BI = R_x @ R_y @ R_z
        self.IB = BI.T
if __name__ ==  "__main__":
    rb = RigidBody()
    rb.updateIB([0,30,0])
    
    rb.printDcm(rb.IB, "[IB]")