
#%% Code Block 1-2
from ros_shim import rospy
from ros_shim import std_msgs, nav_msgs
import time
import math
                    
# ROS initialization
nodeName = "Lab 5 Node"
rospy.init_node(nodeName)

#%% Code Block 1-3
# class for the AGV state machine to move the vehicle along the track
class AgvFsm:
    
   
    # sets the angle of the vehicle
    def update_angle_position(self, data):
        self.theta = data

    # sets the displacement of the vehicle from the track
    def update_displacement(self, data):
        self.displacement = data

    # class initialization
    def __init__(self):
        self.currentState = "stop" #current state based on the state machine
        self.active = True #controls the vehicle to stop and go
        self.theta = 0 
        self.displacement = 0
        self.clockwise = True #if the vehicle is moving in the clockwise or counterclockwise direction on the track
        self.turn = False #instruct if the vehicle to turn around 
        self.agvanglesub = rospy.Subscriber("/1/AGVanglePosition", std_msgs.Float64, self.update_angle_position)
        self.displacementsub = rospy.Subscriber("/1/displacement", std_msgs.Float64, self.update_displacement)

        self.xVel = rospy.Publisher("/1/AGVxVelocity", std_msgs.Float64)
        self.yVel = rospy.Publisher("/1/AGVyVelocity", std_msgs.Float64)
        self.angVel = rospy.Publisher("/1/AGVangularVelocity", std_msgs.Float64)
        
    #updates the main state machine
    def update(self):
        print("Current AGV State: ", self.currentState)
#%% Code Block 1-4
AgvFsmInst = AgvFsm()

for x in range(0, 100): #change range as neede to test
    AgvFsmInst.update()
    time.sleep(0.1)

    
AgvFsmInst.agvanglesub.unregister()
AgvFsmInst.displacementsub.unregister()

#%% Code Block 1-5
    if self.currentState == "turn":
        # publish movements
        pass
    # stop
    elif self.currentState == "stop":
        # publish movements
        pass

    # straight
    elif self.currentState == "straight":
        # publish movements
        pass

    # left
    elif self.currentState == "left":
        # publish movements
        pass

    # right
    elif self.currentState == "right":
        # publish movements
        pass
    
#%% Code Block 1-6
    # publish movements
    self.xVel.publish(0)
    self.yVel.publish(0)
    self.angVel.publish(0)
    # state transitions
    if self.active == True:
        self.currentState = "straight"
        
#%% Code Block 1-7
#constants given in lab background
ANG_VEL = math.pi/16
LIN_VEL = 30
EPSILON1 = 7
EPSILON2 = 10

# global variable for timekeeping
global t
t = 0

# function to keep track of time
def update_t(data):
    global t
    t += 0.1

rospy.Subscriber("/time", std_msgs.Float64, update_t)

#%% Code Block 1-8
    # publish movements
    self.xVel.publish(LIN_VEL*math.cos(self.theta))
    self.yVel.publish(-LIN_VEL*math.sin(self.theta))
    self.angVel.publish(0)
    # state transitions
    if self.active == False:
        self.currentState = "stop"
    elif self.displacement > EPSILON2:
        self.currentState = "right"
        t = 0
    elif self.displacement < -EPSILON2:
        self.currentState = "left"
        t = 0
        
#%% Code Block 1-9
# left
    elif self.currentState == "left":
        # publish movements
        self.xVel.publish(LIN_VEL/(1+math.exp(-t+5))*math.cos(self.theta))
        self.yVel.publish(-LIN_VEL/(1+math.exp(-t+5))*math.sin(self.theta))
        # state transitions
        if self.active == False:
            self.currentState = "stop"
        elif abs(self.displacement) < EPSILON1:
            self.currentState = "straight"
        elif self.displacement > EPSILON2:
            self.currentState = "right"
            t = 0
    # right
    elif self.currentState == "right":
        # publish movements
        self.xVel.publish(LIN_VEL/(1+math.exp(-t+5))*math.cos(self.theta))
        self.yVel.publish(-LIN_VEL/(1+math.exp(-t+5))*math.sin(self.theta))
        # state transitions
        if self.active == False:
            self.currentState = "stop"
        elif abs(self.displacement) < EPSILON1:
            self.currentState = "straight"
        elif self.displacement < -EPSILON2:
            self.currentState = "left"
            t = 0
            
#%% Code Block 1-10
if self.currentState == "turn":
        # publish movements
        self.xVel.publish(0)
        self.yVel.publish(0)
        self.angVel.publish(ANG_VEL)
        # state transitions
        if abs(self.theta - self.startingTheta) > math.pi:
            self.currentState = "stop"
            self.clockwise = not(self.clockwise)
            
#%% Code Block 1-11
    #have the vehicle enter the turn state
    if self.turn == True:
        self.currentState = "turn"
        self.turn = False #turn off so it doesn't keep resetting the angle/re-entering the state
        self.startingTheta = self.theta
        
#%% Code Block 1-12
    if self.clockwise:
        self.angVel.publish(ANG_VEL) #normally
    else:
        self.angVel.publish(-ANG_VEL) #counterclockwise
        
#%% Code Block 1-13
# SchedulingFsm - state machine to select the next package based on scheduling
class SchedulingFsm:
    
    #retrieves the package queue
    def add_request(self,data):
        self.deliveryList = data
        
    # class initialization
    def __init__(self):
        self.currentState = "idle" #state of the SS
        self.deliveryList = [] #deliveries 
        self.delivered = False #used to determine if the delivery is done, and it should move back to the idle state
        self.requestsub = rospy.Subscriber("/request", nav_msgs.Delivery, self.add_request)

    # update - updates the current state and outputs
    def update(self):
        pass
    
#%% Code Block 1-14
SchedulingFsmInst = SchedulingFsm()

for x in range(0, 100): #place your previous while loop at the end of your code with this one
    AgvFsmInst.update()
    time.sleep(0.1)
    SchedulingFsmInst.update()
    
AgvFsmInst.agvanglesub.unregister()
AgvFsmInst.displacementsub.unregister()

SchedulingFsmInst.requestsub.unregister()
    
#%% Code Block 1-15
# update SS state machine
    def update(self):
        if self.currentState == "idle":
            #TODO: set the DS state machine's current package to be executed to None/0
            # next state logic
            if len(self.deliveryList) > 0:
                self.currentState = "serving"
        elif self.currentState == "serving":
            # next state logic
            if len(self.deliveryList) == 0:
                self.currentState = "idle"
            # determine which package to schedule
            else:
                deliveryToExecute = self.deliveryList[0] #the package to deliver
                #TODO: pass the delivery to execute to the DS state machine.
                self.currentState = "wait"
        elif self.currentState == "wait":
            # wait for delivery to complete
            if self.delivered:
                self.currentState = "idle"
                self.delivered = False
                
