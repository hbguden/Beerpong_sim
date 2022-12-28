import numpy as np
class ball:
    def __init__(self,position=[0,0]):
        self.position=np.asarray(position)
        self.x=position[0]
        self.y=position[1]
        self.velocity=np.zeros((2)) #m/s
        self.grab=False
        self.static=True

    def set_pos(self,position,dt=0):
        position=np.asarray(position)
        if dt!=0:
            self.velocity=(position-self.position)/dt
        self.position=position
        self.x=position[0]
        self.y=position[1]
