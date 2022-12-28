# Beerpong_sim
simulator for beerpong

#To import the environment
Import from beerpong_bot import beerpong_bot
code example is located in the bottom of beerpong_bot.py
beerpong_bot can be initialized with ether render_mode="human" or " " and it contains the methods:
reset()
returns next_state, reward, terminated

step(grip, theta ) : takes in (grip:boolean, theta :angles in radians) grip: if we want to grab or not. theta: angle for each joint
returns next_state, reward, terminated


next_state: np.array containing Joint_angles, endeffector x and y, velocity endeffector vx and vy, distance to closest cup from base, angle to closest cup from base, distance to ball from endeffector, 1 if we hold the ball else 0

For more documentation, read in the individual files (start with beerpong_bot.py)
