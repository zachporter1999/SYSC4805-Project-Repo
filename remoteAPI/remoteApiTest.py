# Remote API Test

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import json
import time
import sys

"""
    Parses the json config file

    Return:
        A dictionary containing the configs.
"""
def parseConfig(config_filename):
    f = open(config_filename)
    config = json.load(f)
    f.close()

    return config

# ========================================
# Main
# ========================================
if __name__ == '__main__':
    print("Application Starting...")

    # --------------------
    # Load Config
    # --------------------

    try:
        config = parseConfig("config.json")
    except  FileNotFoundError:
        # No Config file
        print("Error: File not found")
        sys.exit(1)
    except json.JSONDecodeError:
        # Json failed to load
        print("Error: JSON unable to load")
        sys.exit(2)

    # --------------------
    # Remote API setup
    # --------------------

    sim.simxFinish(-1) # Close all opened connections.
    clientID = sim.simxStart(
            config["remoteAPI"]["address"],
            config["remoteAPI"]["port"],
            True,
            True, 
            config["remoteAPI"]["timeout"],
            5)

    # --------------------
    # Get Left Motor Handler
    # --------------------

    res, leftMotor = sim.simxGetObjectHandle(
            clientID, 
            config["leftMotor"], 
            sim.simx_opmode_blocking)

    if res != sim.simx_return_ok:
        print("Failed to find left motor")
        sys.exit(-1)

    # --------------------
    # Get Right Motor Handler
    # --------------------
    
    res, rightMotor = sim.simxGetObjectHandle(
            clientID, 
            config["rightMotor"], 
            sim.simx_opmode_blocking)

    if res != sim.simx_return_ok:
        print("Failed to find right motor")
        sys.exit(-1)

    # --------------------
    # Main Loop
    # --------------------

    sim.simxSetJointTargetVelocity(
            clientID, 
            leftMotor, 
            config["motorSpeed"], 
            sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(
            clientID, 
            rightMotor, 
            config["motorSpeed"]/2, 
            sim.simx_opmode_blocking)
    time.sleep(0.1)

    sim.simxFinish(clientID)    
