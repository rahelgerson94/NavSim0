import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, tan, pi, arctan2, arcsin, deg2rad, rad2deg
from numpy import zeros, array
from numpy.linalg import norm, inv
import Vehicle

import inspect
def printVehicleStates(listOfVehicleObjs, listOfVehicleNames):
    print('---------', end='')
    [print(x, end='\t\t' if i < len(listOfVehicleNames) - 1 else '') for i, x in enumerate(listOfVehicleNames)]

    print('---------')
    print()
    print("rInI:", end = '\t')
    [print(", ".join([f"{x:.2f}" for x in v.rInI]), end='\t\t') for v in listOfVehicleObjs]

    print()
    
    print("vInI:", end = '\t')
    [print(", ".join([f"{x:.2f}" for x in v.vInI]), end='\t\t') for v in listOfVehicleObjs]

    print()
    
    print("aInI:", end = '\t')
    [print(", ".join([f"{x:.2f}" for x in v.aInI]), end='\t\t') for v in listOfVehicleObjs]

    print()
    
    for v in listOfVehicleObjs:
        if hasattr(v, 'angleFromHorizontalRad'):
            print(f"θ (deg): {rad2deg(v.angleFromHorizontalRad):.2f}", end='\t\t\t')
        if hasattr(v, 'betaRad'):
            print(f"{rad2deg(v.betaRad):.2f}", end='\t\t')
    print()
    
def printArray(arr):
    pass

def enter():
    # Get the current stack frame of the caller
    frame = inspect.currentframe().f_back
    function_name = frame.f_code.co_name  # Get the function name
    # Get the class name, if the function is part of a class
    cls_name = frame.f_locals.get('self', None).__class__.__name__ if 'self' in frame.f_locals else ''
    
    print(f"{cls_name}.{function_name}(): entering...")
    
def dbprint(*args):
    # Join the arguments with a space and prepend a tab
    print('\t', *args)
def printDcm(dcm, title = ""):
    
    """
    Utility function to print a Direction Cosine Matrix (DCM) with elements rounded to 2 decimal places.

    Parameters:
        dcm: A 3x3 numpy array representing the DCM.
    """
    if dcm.shape != (3, 3):
        raise ValueError("Input must be a 3x3 matrix.")

    print(f"---- {title} ---")
    for row in dcm:
        print("  ".join(f"{elem:.2f}" for elem in row))
