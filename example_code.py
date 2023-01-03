from beerpong_bot import beerpong_bot
import numpy as np

#make the environment
env=beerpong_bot(render_mode="human")
#env=beerpong_bot(render_mode="")

#get observation
observation = env.reset()

#varuable for sum of reward
fitness=0
terminated=False

while not terminated:
    #using np.random for random numbers, but you should use a network to send inputs derrived from the observations
    observation, reward, terminated= env.step(np.array([np.random.rand()-0.5, np.random.rand(), np.random.rand()])) #observation is a np.array with floats of length 10. we send in 3 parameters.
    fitness+=reward #sum up reward
print(fitness)
