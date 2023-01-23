# Beerpong_sim
simulator for beerpong

Libraries needed: Pygame, numpy

#To import the environment
Import from beerpong_bot import beerpong_bot
code example is located in the bottom of beerpong_bot.py
beerpong_bot can be initialized with ether render_mode="human" or " " and it contains the methods:
reset()
returns next_state,

step(action ) : takes in (grip : float, theta :angles between 0 and 1) grip: bigger than grip>0 if we want to grab, grab<0 if not. theta: angle for each joint
returns next_state, reward, terminated


next_state: np.array containing Joint_angles, endeffector x and y, velocity endeffector vx and vy, distance to closest cup from base, angle to closest cup from base, distance to ball from endeffector, 1 if we hold the ball else 0

The base case for next_state is a np.array with length of 10

It is solved when you have a total reward of 165 (168.3 is max)

For more documentation, read in the individual files (start with beerpong_bot.py)
