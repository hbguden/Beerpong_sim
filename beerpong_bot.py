import numpy as np
from ball import ball
from dof import dof
from cup import cup
from ground import ground
class beerpong_bot():

    def __init__(self, render_mode=""):

        self.dt=1/120 #120hz sim
        self.dof_colours=[(104,149,197),(7,87,152)]
        self.manipulator=dof(2,[0.2,0.2], [0.5,0.5], max_speed=[5.8*self.dt,5.8*self.dt])
        self.ball=ball([0.9,0.5])
        glass=cup(position=[1.65,0.55])
        self.cups=[glass]
        self.floor=ground([0,0.5] ,[2,0.5])
        self.ball_min_dist=0.05 #distance from cup before we check if colision
        self.dof_with=5
        self.SCALE=200
        self.screen_shape=(600,400)
        self.GRAVITY=9.81 #m/s^2
        self.dampening_scale0=1.0 #scale for dampening ortogonal on vector that is hit
        self.dampening_scale1=0.9 #scale for the whole velocity when we hit something
        self.in_cup=2
        self.cup_filled_pos=None
        self.min_cup_dist=np.inf
        self.cup_th=0
        self.deltaX_ball=0.2

        #visual
        self.render_mode=render_mode
        if render_mode == "human":
            import pygame
            pygame.init()
            self.clock = pygame.time.Clock()
            self.screen = pygame.display.set_mode(self.screen_shape)
            pygame.display.set_caption('Simulation')
            self.pygame=pygame

        #set value for closest glass
        for glass in self.cups:
            dist=np.sqrt((glass.x-self.manipulator.position[0])**2 + (glass.y-self.manipulator.position[1])**2)
            if self.min_cup_dist<dist:
                self.min_cup_dist=dist
                self.cup_th=np.arctan2(glass.y-self.manipulator.position[1],glass.x-self.manipulator.position[0])

    def reset(self):
        self.__init__(render_mode=self.render_mode)
        next_state=[]
        for i in range(self.manipulator.joint_amount):
            next_state.append(0)
        endeffector_pos=self.manipulator.getJoint_position(self.manipulator.joint_amount)  #add forward kinematics
        next_state.append(endeffector_pos[0,3])
        next_state.append(endeffector_pos[1,3])
        next_state.append(0)
        next_state.append(0)
        next_state.append(self.min_cup_dist)
        next_state.append(self.cup_th)
        next_state.append(np.sqrt((endeffector_pos[0,3]-self.ball.x)**2 + (endeffector_pos[0,3]-self.ball.x)**2)) #distance to ball endeffector
        next_state.append(int(self.ball.grab)) #if we have the ball
        return (next_state, 0, False)

    def drawDOF(self, dof, width, colours, screen):
        #draws manipulator on screen
        pos0=dof.getJoint_position(0)
        pos0=(pos0[0,3]*self.SCALE, self.screen_shape[1]-pos0[1,3]*self.SCALE)
        for i in range(dof.joint_amount):
            pos1=dof.getJoint_position(i+1)
            pos1=(pos1[0,3]*self.SCALE, self.screen_shape[1]-pos1[1,3]*self.SCALE)
            self.pygame.draw.line(screen,colours[0], pos1, pos0, width)
            self.pygame.draw.circle(screen,colours[1], pos0, width)
            pos0=pos1

    def drawBall(self,ball, width, colour, screen):
        #draws ball
        pos=(ball.x*self.SCALE, self.screen_shape[1]-ball.y*self.SCALE)
        self.pygame.draw.circle(screen, (0,0,0), pos, width+1)
        self.pygame.draw.circle(screen, colour, pos, width)

    def drawCup(self,cup,colour,screen):
        #draws cup
        corners=[None]*4
        for i in range(len(corners)):
            corner=cup.corners[i]
            corners[i]=(corner[0]*self.SCALE, self.screen_shape[1]-corner[1]*self.SCALE)
        self.pygame.draw.polygon(screen,colour,corners)

    def draw_floor(self,screen,floor):
        pos0=(int(floor.position[0][0]*self.SCALE), int((self.screen_shape[1] - floor.position[0][1]*self.SCALE/2)))
        pos1=(int(floor.position[1][0]*self.SCALE), int((self.screen_shape[1] - floor.position[1][1]*self.SCALE/2)))
        width=int(floor.position[0][1]*self.SCALE)
        self.pygame.draw.line(screen, floor.colour, pos0, pos1, width)

    def intersect(self,s0, s1): #[[x,y],[x,y]]
        #returns point where they intersect
        dx0 = s0[1][0]-s0[0][0]
        dx1 = s1[1][0]-s1[0][0]
        dy0 = s0[1][1]-s0[0][1]
        dy1 = s1[1][1]-s1[0][1]
        p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
        p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
        p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
        p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
        if not ( (p0*p1<=0) & (p2*p3<=0)):
            return None
        #they intersect, time to find out where!
        c1=dy0*s0[0][0] + dx0*s0[0][1]
        c2=dy1*s0[1][0] + dx1*s0[1][1]
        det=dy0*dx1 - dy1*dx0
        if det==0:
            #paralell. just check what is closest to
            if ((s0[0][0]-s1[0][0])**2 + (s0[0][1]-s1[0][1])**2) < ((s0[0][0]-s1[1][0])**2 + (s0[0][1]-s1[1][1])**2):
                return np.asarray([s1[0][0],s1[0][1]])
            else:
                return np.asarray([s1[1][0],s1[1][1]])
        else:
            x = (dx1*c1 - dx0*c2)/det
            y = (dy0*c2 - dy1*c1)/det
            return np.asarray([x,y])

    def bounce(self, vector1, vector2):
        #finds vector to follow when we bounce on a wall
        vec1=np.asarray(vector1)
        vec2=np.asarray(vector2)
        vec1_proj=vec2*(np.dot(vec1,vec2)/np.dot(vec2,vec2)) #projection of vec1 on vec2
        vec1_proj90=vec1-vec1_proj #projection of vec1 90 deg on vec2
        vec_out=vec1_proj90*self.dampening_scale0 + vec1_proj #sum up the vectors and scale with a dampening
        return vec_out/(np.sqrt(np.dot(vec_out,vec_out))) #returns normalized vector


    def my_eval(self,grip, *theta):
        #takes boolean and desired theta
        #returns (next_state : np.array, reward, terminated)
        reward=0
        endeffector_start=self.manipulator.getJoint_position(self.manipulator.joint_amount)
        if not grip: #release ball
            self.ball.grab=False
        if (grip and (not self.ball.grab)): #check if we can grab ball
            if ((endeffector_start[0,3]-self.ball.x)**2 + (endeffector_start[1,3]-self.ball.y)**2)<self.manipulator.grab_dist**2:
                self.ball.grab=True
        index=0
        for angle in theta: #move the manipulator
            reward-=self.manipulator.move(angle,index)*3
            index+=1
        endeffector_pos=self.manipulator.getJoint_position(self.manipulator.joint_amount) #get position of endeffector
        if self.ball.grab: #move ball if we grabbed it
            self.ball.set_pos([endeffector_pos[0,3],endeffector_pos[1,3]],self.dt)
            self.ball.static=False
        elif (self.ball.static==False): #move ball with physics!
            #calculate position with gravity!
            pos_ball=[0,0]
            pos_ball[0]+=self.ball.velocity[0]*self.dt+self.ball.x
            pos_ball[1]+=(self.ball.velocity[1]-self.GRAVITY*self.dt)*self.dt + self.ball.y
            #check if i hit someting!
            ball_hit=False
            for glass in self.cups:
                #check if we are close to the cup
                if ((self.ball.x + self.ball_min_dist > glass.corners[3][0]) and (self.ball.y - self.ball_min_dist < glass.corners[3][1])) and ((self.ball.y + self.ball_min_dist > glass.corners[1][1]) and (self.ball.x - self.ball_min_dist < glass.corners[1][0])):
                    for i in range(len(glass.corners)-1):
                        intersection=self.intersect([self.ball.position,pos_ball],[glass.corners[i],glass.corners[i+1]]) #check colition with cup

                        if (intersection is not None): #and ball_hit!=True:
                            #we hit something
                            ball_hit=True
                            #calculate vector for bounce and velocity
                            direction=self.bounce([self.ball.x-intersection[0],self.ball.y-intersection[1]],glass.corners[i]-glass.corners[i+1])
                            overshoot_frame=pos_ball-intersection #how much we passed the wall by
                            pos_ball=intersection + (direction*np.sqrt(np.dot(overshoot_frame,overshoot_frame)))#sets position after bounce
                            self.ball.velocity=direction*(np.sqrt(np.dot(self.ball.velocity,self.ball.velocity)))*self.dampening_scale1 #set the velocity, and scale it
                    if (self.intersect([self.ball.position,pos_ball],[glass.corners[3],glass.corners[0]]) is not None):
                        self.cup_filled_pos=np.asarray([glass.x,glass.y])

            if (pos_ball[1] < self.floor.position[0][1]) and (pos_ball[0] > self.floor.position[0][0]) and (pos_ball[0] < self.floor.position[1][0]):  #check for collition with floor
                #print("calcint")
                intersection=self.intersect([self.ball.position,pos_ball],self.floor.position)
                if (intersection is not None):
                    # we hit the floor
                    ball_hit=True
                    #calculate vector for bounce and velocity
                    direction=self.bounce([self.ball.x-intersection[0],self.ball.y-intersection[1]],(self.floor.position[1][0]-self.floor.position[0][0], self.floor.position[1][1]-self.floor.position[0][1]))
                    #print("dir: ",direction)
                    overshoot_frame=pos_ball-intersection #how much we passed the wall by
                    pos_ball=intersection + (direction*np.sqrt(np.dot(overshoot_frame,overshoot_frame)))#sets position after bounce
                    #print("position", pos_ball)
                    self.ball.velocity=direction*(np.sqrt(np.dot(self.ball.velocity,self.ball.velocity)))*self.dampening_scale1 #set the velocity, and scale it
                    #print("vel",self.ball.velocity)
            if (self.cup_filled_pos is not None) and self.in_cup!=0:
                self.in_cup-=1
            if self.in_cup==0:
                reward+=100
                self.ball.set_pos(self.cup_filled_pos)
            elif ball_hit:
                self.ball.set_pos(pos_ball)
            else:
                self.ball.set_pos(pos_ball, self.dt)
        next_state=[] # joint angles, endeffector pos, distance and angle towards closest cup (operation space) , distance to ball endeffector
        for index in range(self.manipulator.joint_amount):
            next_state.append(self.manipulator.theta[index]) #add thetas
        endeffector_pos_new=self.manipulator.getJoint_position(self.manipulator.joint_amount)  #add forward kinematics
        next_state.append(endeffector_pos_new[0,3])
        next_state.append(endeffector_pos_new[1,3])
        next_state.append(endeffector_pos_new[0,3]-endeffector_pos[0,3]) # velocityX
        next_state.append(endeffector_pos_new[1,3]-endeffector_pos[0,3]) # velocityY
        next_state.append(self.min_cup_dist)
        next_state.append(self.cup_th)
        next_state.append(np.sqrt((endeffector_pos_new[0,3]-self.ball.x)**2 + (endeffector_pos_new[0,3]-self.ball.x)**2)) #distance to ball endeffector
        next_state.append(int(self.ball.grab)) #if we have the ball

        #calculate reward:
        if (self.ball.grab==False):
            if self.ball.x>self.deltaX_ball:
                if (self.ball.y>self.cups[0].corners[0][1]):
                    if self.ball.x<self.cups[-1].corners[1][0]: # add reward while we are in front of the last cup
                        reward+=50*(self.ball.x-self.deltaX_ball)
                    else:
                        reward+=50*(self.cups[-1].corners[1][0]-self.ball.x)
                else:
                    if self.ball.x<self.cups[-1].corners[1][0]: # add reward while we are in front of the last cup
                        reward+=20*(self.ball.x-self.deltaX_ball)
                    else:
                        reward+=20*(self.cups[-1].corners[1][0]-self.ball.x)
                self.deltaX_ball=self.ball.x

        return (np.asarray(next_state), reward, self.ball.y<0 or self.in_cup==0)

    def step(self, grip, *theta):
        if self.render_mode=="human":
            self.screen.fill((255,255,255))
            self.clock.tick(60)
            self.draw_floor(self.screen,self.floor)
            self.drawDOF(self.manipulator, self.dof_with, self.dof_colours, self.screen)
            self.drawBall(self.ball,5,(255,255,255), self.screen)
            self.drawCup(self.cups[0],(255,0,0),self.screen)
            self.pygame.event.pump()
            self.pygame.display.flip()
        return self.my_eval(grip, *theta)


#testcode
if __name__ == "__main__":
    env=beerpong_bot(render_mode="human")
    #env=beerpong_bot(render_mode="")
    data, reward, t = env.reset()
    fitness=0
    while not t:
        grabit=np.random.rand()>0.5
        data, reward, t= env.step(grabit, np.random.rand()*np.pi*2, np.random.rand()*np.pi*2)
        fitness+=reward
    print(fitness)
