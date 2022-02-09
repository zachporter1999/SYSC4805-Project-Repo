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

import time
import sys

coppeliaServerAddress = '127.0.0.1'
coppeliaServerPort    = 19999

# ========================================
# Main
# ========================================
if __name__ == '__main__':
    print("Application Starting...")

    sim.simxFinish(-1) # Close all opened connections.
    clientID = sim.simxStart(coppeliaServerAddress, coppeliaServerPort, True, True, 5000, 5)

    res, leftMotor = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)

    if res != sim.simx_return_ok:
        print("Failed to find left motor")
        sys.exit(-1)
    
    res, rightMotor = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)

    if res != sim.simx_return_ok:
        print("Failed to find right motor")
        sys.exit(-1)

    motorSpeed = 10

    for itr in range(0, 100):
        sim.simxSetJointTargetVelocity(clientID, leftMotor, motorSpeed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, -motorSpeed, sim.simx_opmode_blocking)
        time.sleep(0.1)

    sim.simxFinish(clientID)    
