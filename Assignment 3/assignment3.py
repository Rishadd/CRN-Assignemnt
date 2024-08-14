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

import numpy
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
global waypoints
def aruco(img_flip):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img_flip)
    corners = np.array(markerCorners)
    angle, avgX, avgY = 0, 0, 0

    if len(corners) != 0:
        c1 = [int(corners[0, 0, 0, 0]), int(corners[0, 0, 0, 1])]
        c4 = [int(corners[0, 0, 3, 0]), int(corners[0, 0, 3, 1])]

        for n in range(0, 4):
            # calculating the coordinates of the bot
            avgX += int(corners[0, 0, n, 0])
            avgY += int(corners[0, 0, n, 1])

        # find slope of the lines
        angle = 0

        if (c1[0] - c4[0]) == 0:
            angle = 0
        else:
            slopeBot = (c1[1] - c4[1]) / (c1[0] - c4[0])
            #print("Slope bot: ", slopeBot)
            angle = np.arctan((-slopeBot))
            angle = -int(((angle * 180) // 3.14) - 90)
            #print("Angle: ", angle)

        avgX //= 4
        avgY //= 4

    return (avgX, avgY)


def getWaypoints(img_flip):
    global waypoints
    img_grey = cv2.cvtColor(img_flip, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.medianBlur(img_grey, 5)

    param1 = 200  # cv2.getTrackbarPos("Param1", "Circle Parameters")
    param2 = 15  # cv2.getTrackbarPos("Param2", "Circle Parameters")
    rMin = 0  # cv2.getTrackbarPos("min radius", "Circle Parameters")
    rMax = 10  # cv2.getTrackbarPos("max radius", "Circle Parameters")

    # finding the waypoints
    rows = img_blur.shape[0]
    circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=param1, param2=param2,
                               minRadius=rMin, maxRadius=rMax)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        waypoints = [(circles[0, 6, 0], circles[0, 6, 1]),
                     (circles[0, 4, 0], circles[0, 4, 1]),
                     (circles[0, 3, 0], circles[0, 3, 1]),
                     (circles[0, 2, 0], circles[0, 2, 1]),
                     (circles[0, 1, 0], circles[0, 1, 1]),
                     (circles[0, 7, 0], circles[0, 7, 1]),
                     (circles[0, 5, 0], circles[0, 5, 1]),
                     (circles[0, 0, 0], circles[0, 0, 1])]

        count = 0
        for center in waypoints:
            # center = (i[0], i[1])
            # circle center
            # cv2.circle(img_flip, center, 10, (0, 0, 0), cv2.FILLED)
            cv2.putText(img_flip, str(count), center, cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 2)
            count += 1

    return waypoints


def getImage():
    camera = sim.getObjectHandle("/vision_sensor")

    # Reading video feed from the vision sensor
    simImg, simResolution = sim.getVisionSensorImg(camera)  # gets image sequence from vision sensor
    img = np.frombuffer(simImg, dtype=np.uint8)  # converts bit stream to unsigned integers
    img = img.reshape((simResolution[1], simResolution[0], 3))  # idk why this is there
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img_bgr = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
    img_flip = cv2.flip(img_bgr, 1)

    #cv2.imshow("Vision sensor", img_flip)

    #cv2.waitKey(1)

    return img_flip


def correctAngle(pt, img_flip):
    print("Entered angle function")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img_flip)
    corners = np.array(markerCorners)
    angle, avgX, avgY = 0, 0, 0

    if len(corners) != 0:
        c1 = [int(corners[0, 0, 0, 0]), int(corners[0, 0, 0, 1])]
        c4 = [int(corners[0, 0, 3, 0]), int(corners[0, 0, 3, 1])]
        print("C1:",c1,"C2:",c4)

        if (c1[0] - c4[0]) == 0 or (pt[0] - c4[0]) == 0:
            angle = 90
        else:
            slopeBot = (c1[1] - c4[1]) / (c1[0] - c4[0])
            slopeLine = (pt[1] - c4[1]) / (pt[0] - c4[0])
            print("Slope bot: ", slopeBot)
            angle = np.arctan((slopeLine - slopeBot) / (1 + (slopeBot * slopeLine)))
            angle = -int(((angle * 180) // 3.14) - 90)
            print("Angle: ", angle)

        kp = 0.02  # Proportionality constant
        vel_limit = 1
        while angle != 0:
            print("Current angle loop!")
            print("Angle:",angle)
            if angle >= 0 and angle < 90:
                vel = kp * angle
                if vel > vel_limit:
                    vel = vel_limit
                print("Velocity:", vel)
                sim.setJointTargetVelocity(int(r_joint), vel)
                sim.setJointTargetVelocity(int(l_joint), vel)
            elif angle >= 90 and angle < 180:
                vel = kp * angle
                if vel > vel_limit:
                    vel = vel_limit
                print("Velocity:", vel)
                sim.setJointTargetVelocity(int(r_joint), -vel)
                sim.setJointTargetVelocity(int(l_joint), -vel)
            if (c1[0] - c4[0]) == 0 or (pt[0] - c4[0]) == 0:
                angle = 90
            else:
                slopeBot = (c1[1] - c4[1]) / (c1[0] - c4[0])
                slopeLine = (pt[1] - c4[1]) / (pt[0] - c4[0])
                angle = np.arctan((slopeLine - slopeBot) / (1 + (slopeBot * slopeLine)))
                angle = -int(((angle * 180) // 3.14) - 90)
        vel = 0
        sim.setJointTargetVelocity(int(r_joint), vel)
        sim.setJointTargetVelocity(int(l_joint), -vel)
        return


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

    global waypoints
    global r_joint
    global l_joint
    r_joint = sim.getObject("/crn_bot/joint_r")
    l_joint = sim.getObject("/crn_bot/joint_l")

    img = getImage()
    X, Y = aruco(img)
    waypoints = getWaypoints(img)

    while True:
        kp = 1  # Proportionality constant
        vel_limit = 3

        for point in waypoints:
            dist = (point[0] - X) ^ 2 + (point[1] - Y) ^ 2
            edist = np.sqrt(dist)

            while edist > 4:
                vel = kp * edist
                if vel > vel_limit:
                    vel = vel_limit
                print("Velocity:", vel)
                sim.setJointTargetVelocity(int(r_joint), vel)
                sim.setJointTargetVelocity(int(l_joint), -vel)
                img = getImage()
                X, Y = aruco(img)
                print("X:", X, "Y:", Y)
                dist = (point[0] - X) ^ 2 + (point[1] - Y) ^ 2
                edist = np.sqrt(dist)
                print("Dist:", dist)
                print("Error:", edist)
                print()

            #STOP THE BOT!!!
            vel = 0
            sim.setJointTargetVelocity(int(r_joint), vel)
            sim.setJointTargetVelocity(int(l_joint), -vel)
            print("Velocity:", vel)
            #break
            #ROTATE FFS
            correctAngle(point, img)
            time.sleep(2)

        #crop = cv2.resize(img_flip, (700, 700))
        #cv2.imshow("Vision sensor", crop)

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
