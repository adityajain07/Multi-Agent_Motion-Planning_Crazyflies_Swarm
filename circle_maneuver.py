from pycrazyswarm import Crazyswarm
import math
import sys

TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.5

# Wait time between each circle to launch
WAIT = 1.0

# Time to hover after launch before moving in a circle
HOVER_DURATION = 5.0

# Time to move in a circle
FLIGHT_DURATION = 30

# Time to move from one point to another in a circle
MOVE_TIME = 0.5

# How many radians to move in a circle
ANGLE_INCREMENT = 0.1

# Drone ID for each circle
top_circle = [1]
middle_circle = [2,3,4,5]
bottom_circle = [6,7,8,9,10,11,12,13]

# List of drones to fly based on command line arguments
fly_list = []

# Check if simulation mode is on
is_Simulation = False
if (sys.argv[1] =='--sim'):
    print("Running in Simulation Mode")
    is_Simulation = True

# Choose which circle to fly based on command line arguments
circle_to_fly = ''

# If simulation mode is on, then the third argument is the circle to fly
# If simulation mode is off, then the second argument is the circle to fly
# If no specific circle to fly is given, then all circles are flown
if (is_Simulation):
    if (len(sys.argv) == 3):
        circle_to_fly = sys.argv[2]
    else:
        circle_to_fly = ''
else:
    if (len(sys.argv) == 2):
        circle_to_fly = sys.argv[1]
    else:
        circle_to_fly = ''

if (circle_to_fly=='1'):
    print("Only Flying Bottom Circle!")
    fly_list = bottom_circle
elif (circle_to_fly=='2'):
    print("Only Flying Middle Circle!")
    fly_list = middle_circle
elif (circle_to_fly=='3'):
    print("Only Flying Top Circle!")
    fly_list = top_circle
else:
    fly_list.extend(top_circle)
    fly_list.extend(middle_circle)
    fly_list.extend(bottom_circle)

# Height of each circle (in meters)
top_circle_height = 3
middle_circle_height = 2
bottom_circle_height = 1

def main():
    swarm = Crazyswarm()
    craziflies = swarm.allcfs.crazyflies
    timeHelper = swarm.timeHelper
    
    # Set group mask for each circle
    for cf in craziflies:
        if cf.id in top_circle:
            cf.setGroupMask(1)
        elif cf.id in middle_circle:
            cf.setGroupMask(2)
        elif cf.id in bottom_circle:
            cf.setGroupMask(4)

    # Take off each circle in order and wait for each to finish
    for cf in craziflies:
        if cf.id in fly_list:
            cf.takeoff(targetHeight=top_circle_height, duration=TAKEOFF_DURATION, groupMask=1)
            timeHelper.sleep(TAKEOFF_DURATION)
            cf.takeoff(targetHeight=middle_circle_height, duration=TAKEOFF_DURATION, groupMask=2)
            timeHelper.sleep(TAKEOFF_DURATION)
            cf.takeoff(targetHeight=bottom_circle_height, duration=TAKEOFF_DURATION, groupMask=4)
            timeHelper.sleep(TAKEOFF_DURATION)

    # After all circles have taken off, wait for a bit
    timeHelper.sleep(HOVER_DURATION)

    # Start maneuver to move each drone in a circle
    initial_time = 0
    while (initial_time<FLIGHT_DURATION):
        for cf in craziflies:
            if cf.id in fly_list:
                # Don't move the first drone
                if cf.id != 1:
                    radius = math.sqrt(cf.position()[0]**2 + cf.position()[1]**2)
                    current_angle = round (math.atan2(cf.position()[1],cf.position()[0]),2)
                    new_angle = round(current_angle + ANGLE_INCREMENT,2)
                    new_x = round(radius*math.cos(new_angle),2)
                    new_y = round(radius*math.sin(new_angle),2)
                    new_z = round(cf.position()[2],2)
                    cf.goTo([new_x, new_y, new_z], 0, MOVE_TIME)
                    # TODO - Add a check to make sure the drone has reached the new position
            
        timeHelper.sleep(0.5)
        # Increment timer to be MOVE TIME + SLEEP TIME (0.5)
        initial_time += (0.5+MOVE_TIME)
        
    # After the maneuver is done, land each circle in order and wait for each to finish
    for cf in craziflies:
        if cf.id in fly_list:
            cf.land(targetHeight=0.04, duration=LAND_DURATION)
    timeHelper.sleep(5)

if __name__ == "__main__":
    main()
