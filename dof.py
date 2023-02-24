import numpy as np
#code for the manipulator.
#
#when it is initali<ed and all thetas=0: the robot looks like this: o--o--C (2dof)
#constraints are given on the form [(con1), (con2), (con3)] where con1 is a,b where the manipulator cannot move in the angle between a and b
class dof:
    def __init__(self, joints_amount = 2, lengths =[1,1], position= [0,0], max_speed=[0.03,0.03], constraints=[(np.pi,0),(np.pi-0.2, -np.pi+0.2)], movement='legacy'):
        #makes the manipulator.
        time_max_acceleration=10
        self.joint_amount=joints_amount #number of joints
        self.lengths=np.asarray(lengths) #length between joints in meters
        self.theta=np.zeros((joints_amount), dtype=np.float64) #the angle in the joints
        self.position=np.asarray(position) #the position of the base
        self.max_speed=np.asarray(max_speed) #max speed for the manipulator joints. (this is the max a joint can move between every time move() gets called)
        self.grabber=False #if you try to grab something or not
        self.grab_dist=0.02 #the distance from the endpoint that we can grab a object. (grab tolerance)
        self.constraints=[] #if we have some movement constraints for the angles.
        self.movement=movement #legacy, force, velocity
        self.velocity_rad=np.zeros((joints_amount)) #angular velocity of joints
        self.max_acceleration = self.max_speed/time_max_acceleration #max acceleration joints, angular
        if constraints is None: #Set it to None for no constraints
            self.constraints=None
        else:
            for c in constraints: #make sure constraints are in range 0,2pi
                self.constraints.append( (((c[0]%(2*np.pi))+(2*np.pi))%(2*np.pi), ((c[1]%(2*np.pi))+(2*np.pi))%(2*np.pi)))
            if len(self.constraints)<self.joint_amount: #applies the last constrain for the rest of the joints. if you dont want a constraint for a joint it needs to be [(con1),(con2),None,(con4)]
                for i in range(len(constraints),self.joint_amount,1):
                    self.constraints.append(self.constraints[i-1]) #the rest will use the last constrain

    def config(self, length): #it is possible to reconfigure the length with this function
        self.lengths=np.asarray(length)

    def diff_rad(self, th1, th2):
        #returns th2-th1 normalized
        pi2=2*np.pi
        diff=th2-th1
        diff=diff%pi2 # between -2pi and 2pi
        diff+=pi2     # between 0 and 4pi
        diff=diff%pi2 # between 0 and 2pi
        if diff>np.pi:
            diff-=pi2
        #between -pi and pi
        return diff

    def move(self,joint_angle, joint_number):
        #moves the given joint of the manipulator. move (pi, 0) -> will move the first joint towards pi rad
        def constrain(joint_angle, joint_number):
            if np.abs(self.diff_rad(joint_angle,self.constraints[joint_number][0])) < np.abs(self.diff_rad(joint_angle,self.constraints[joint_number][1])):
                joint_angle=self.constraints[joint_number][0]
            else:
                joint_angle=self.constraints[joint_number][1]
            return joint_angle
        #moves the manipulator

        if self.movement=='velocity': #then its the amount we want to move the joint
            joint_angle+=self.theta[joint_number]
        elif self.movement=='force': #give a acceleration
            sign=np.sign(joint_angle)
            joint_angle = sign*np.minimum(self.max_acceleration[joint_number], joint_angle) + self.velocity_rad[joint_number] + self.theta[joint_number] #get the speed
        diff=self.diff_rad(self.theta[joint_number],joint_angle)
        if (diff<-self.max_speed[joint_number]):
            joint_angle=self.theta[joint_number]-self.max_speed[joint_number]
        elif (diff>self.max_speed[joint_number]):
            joint_angle=self.theta[joint_number]+self.max_speed[joint_number]
        joint_angle=joint_angle%(2*np.pi)
        #joint_angle is now between 0 and 2_pi
        if (self.constraints is not None) and (self.constraints[joint_number] is not None):
            if (joint_angle>self.constraints[joint_number][0]):
                if (self.constraints[joint_number][0]<self.constraints[joint_number][1]):
                    if joint_angle<self.constraints[joint_number][1]:
                        #we are in constrained zone
                        joint_angle=constrain(joint_angle,joint_number)
                else: #con0> con1 -> evething bigger than con0 is out
                    #we are in constrained zone
                    joint_angle=constrain(joint_angle,joint_number)
            elif (joint_angle<self.constraints[joint_number][1]) and (self.constraints[joint_number][1]<self.constraints[joint_number][0]): #joint_angle is less than con0 and less then con1:
                #we are in constrained zone
                joint_angle=constrain(joint_angle,joint_number)
        old_th=self.theta[joint_number]
        self.velocity_rad[joint_number]=self.diff_rad(self.theta[joint_number],joint_angle)
        self.theta[joint_number]=joint_angle
        return np.abs(self.diff_rad(old_th,joint_angle))

    def grab(self, grab):
        self.grabber=grab

    def getJoint_position(self, joint_number = 0):
        #returns the position of the joint.
        position=np.matrix([[1,0,0,self.position[0]],
                            [0,1,0,self.position[1]],
                            [0,0,1,0],[0,0,0,1]])
        for i in range(joint_number):
            A=np.matrix([   [np.cos(self.theta[i]),  -np.sin(self.theta[i]),  0,  self.lengths[i]*np.cos(self.theta[i])],
                            [np.sin(self.theta[i]),  np.cos(self.theta[i]),   0,  self.lengths[i]*np.sin(self.theta[i])],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            position=position@A
        return position #position matrix where [0,3] is x and [1,3] is y
