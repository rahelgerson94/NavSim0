import numpy as np
from numpy import zeros 
from RigidBody import RigidBody 
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad
from numpy import zeros 
from numpy.linalg import norm
from Vehicle import Pursuer
from Vehicle import Target
dt = 1/100

if __name__ == "main":
    target = RigidBody()
    pursuer = RigidBody()
    guide = Guidance()

    HErad = np.deg2rad(20)
    tstart = 0
    tend = 100
    dt = .001
    tvec = np.linspace(tstart, tend, tend/dt)
    for i in tvec:
        guidance.update(pursuer,target)
        ap = guidance.getAPureInP()
        pursuer.update()
        target.update()

