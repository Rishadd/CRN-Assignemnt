'''
*****************************************************************************************
*
* All the functions in this file are used to control the robot in the CoppeliaSim
* simulation via APIs
*
*****************************************************************************************
'''

import sys
import traceback
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np


############################## GLOBAL VARIABLES ######################################
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


############################ USER DEFINED FUNCTIONS ##################################

################################ MAIN FUNCTION #######################################

def simulator(sim):
    """
    Purpose:
    ---
    This function should implement the control logic for the given problem statement
    You are required to actuate the rotary joints of the robot in this function, such that
    it does the required tasks.

    Input Arguments:
    ---
    `sim`    :   [ object ]
        ZeroMQ RemoteAPI object

    Returns:
    ---
    None

    Example call:
    ---
    simulator(sim)
    """

    #### YOUR CODE HERE ####

    r_joint = sim.getObject("/crn_bot/joint_r")
    l_joint = sim.getObject("/crn_bot/joint_l")
    sim.setJointTargetVelocity(int(r_joint), 5)
    sim.setJointTargetVelocity(int(l_joint), -10)

    camera = sim.getObjectHandle("/vision_sensor")


    #Setup for reading Aruco markers
    #cap = cv2.VideoCapture(1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    #Reading video feed from the vision sensor
    while True:
        simImg, simResolution = sim.getVisionSensorImg(camera)      #gets image sequence from vision sensor
        img = np.frombuffer(simImg, dtype=np.uint8)                 #converts bit stream to unsigned integers
        img = img.reshape((simResolution[1], simResolution[0], 3))  #idk why this is there
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img_flip = cv2.flip(img_bgr,1)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img_flip)
        corners = np.array(markerCorners)
        avgX, avgY = 0, 0

        if len(corners) != 0:
            c1 = [int(corners[0, 0, 0, 0]), int(corners[0, 0, 0, 1])]
            c2 = [int(corners[0, 0, 1, 0]), int(corners[0, 0, 1, 1])]
            c3 = [int(corners[0, 0, 2, 0]), int(corners[0, 0, 2, 1])]
            c4 = [int(corners[0, 0, 3, 0]), int(corners[0, 0, 3, 1])]
            for n in range(0, 4):
                #putting dots on the corners
                cv2.circle(img_flip, (int(corners[0, 0, n, 0]), int(corners[0, 0, n, 1])), 5, (0, 255, 255), cv2.FILLED)

                #calculating the coordinates of the bot
                avgX += int(corners[0, 0, n, 0])
                avgY += int(corners[0, 0, n, 1])

            #creating a line that follows the robot
            cv2.line(img_flip, (c1[0], c1[1]),
                        (c4[0], c4[1]), (255, 255, 0), 2)
            #creating a reference line
            cv2.line(img_flip, (c1[0], c1[1]),
                     (c1[0], c1[1]-50), (255, 255, 0), 2)
            #find slope of the lines
            angle = 0

            if (c1[0]-c4[0]) == 0:
                pass
            else:
                slopeBot = (c1[1]-c4[1])/(c1[0]-c4[0])
                print("Slope bot: ", slopeBot)
                angle = np.arctan((-slopeBot))
                angle = -int(((angle*180)//3.14)-90)
                print("Angle: ", angle)


            #print("ID: ", markerIds[0, 0])
            avgX //= 4
            avgY //= 4
            cv2.putText(img_flip, "X: "+str(avgX), (15,50), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255), 2)
            cv2.putText(img_flip, "Y: " + str(avgY), (15, 80), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img_flip, "ID:  " + str(markerIds[0, 0]), (15, 110), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img_flip, "Angle: " + str(angle), (15, 140), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
        print()

        crop = cv2.resize(img_flip, (500, 500))
        cv2.imshow("Vision sensor", crop)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    return None


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    try:

        ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the robot navigation logic written by participants
        try:
            simulator(sim)
            time.sleep(5)

        except Exception:
            print('\n[ERROR] Your simulator function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

        ## Stop the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except KeyboardInterrupt:
        ## Stop the simulation using ZeroMQ RemoteAPI
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation interrupted by user in CoppeliaSim.')
        else:
            print('\nSimulation could not be interrupted. Stop the simulation manually .')
            sys.exit()
