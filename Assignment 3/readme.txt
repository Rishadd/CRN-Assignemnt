Okay so, I detected the centres of the red circles using OpenCV functions.
STored them in correct sequential order of navigation in a list called waypoints
Calculated distance between the bot and the waypoint, one at a time and set velocity using a Kp constant
Once the waypoint is reached, make velocity 0 and correct the angle using another Kp constant such that the bot alligns with the next waypoint
Continue the same process till the bot reaches the goal
