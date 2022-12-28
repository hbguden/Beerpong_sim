import numpy as np
class cup:
    def __init__(self,position=[0,0],height=0.12, width=0.08):
        self.x=position[0]
        self.y=position[1]
        self.height=height
        self.width=width
        self.corners=np.array([(self.x+self.width/2,self.y+self.height/2), (self.x+(self.width/2)*0.75,self.y-self.height/2), (self.x-(self.width/2)*0.75,self.y-self.height/2), (self.x-self.width/2,self.y+self.height/2)]) #top right, bottom right, bottom left,top left

    def move(position=[0,0]):
        self.x=position[0]
        self.y=position[1]
        self.corners=np.array([(self.x+self.width/2,self.y+self.height/2), (self.x+(self.width/2)*0.75,self.y-self.height/2), (self.x-(self.width/2)*0.75,self.y-self.height/2), (self.x-self.width/2,self.y+self.height/2)]) #top right, bottom right, bottom left, top left
