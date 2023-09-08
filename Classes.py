# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 09:19:31 2019

@author: rak1618

return (-np.dot(self._pos - other._pos,self._vel-other._vel)+np.sqrt(((np.dot(self._pos - other._pos,self._vel-other._vel))**2)-)
A module containing my classes
"""
import numpy as np
import pylab as pl
import matplotlib as plt
#from numpy import linalg 

class Ball:
    
    def __init__(self, mass=1, radius=1, pos=[0.0,0.0], vel=[0.1,0.1],container=False):
        self._mass=mass
        self._radius=radius
        self._pos=np.array(pos)
#        print(type(self.pos))
        self._vel=np.array(vel)
        self._container=container
        
        if self._radius >= 0:
            k = np.array([0,85, 170])
            np.random.shuffle(k)
            print(k)
            
            h= [np.random.randint(0,85),np.random.randint(0,85),np.random.randint(0,85)]
            for a in range(3):
                h[a] += k[a]
#            r = np.random.randint(0,85)/255
#            g = np.random.randint(0,85)/255
#            b = np.random.randint(0,85)/255
            r = h[0]/255
            g = h[1]/255
            b = h[2]/255
            rgb = [r,g,b]
#            colourlist = ['blue','green','red','purple','redbrick','gray','yellow',]
            self._circ = pl.Circle(self._pos,self._radius,fc=rgb)  
                                   #colourlist[np.random.randint(0,len(colourlist))])
        else:
            self._circ = pl.Circle([0.0,0.0],-1*self._radius,fill=False,fc='g')
        
    '''    if np.linalg.norm(pos) > 9:
            raise Exception("Ball is outside container")    ignore since radius of container can change''' 

    def pos(self):
        return self._pos
    
    def vel(self):
        return self._vel
    
    def get_patch(self):
        return self._circ
        
    def move(self,dt):
        self._pos = self._pos+self._vel*dt
        #working out the lowest time to next collision
        self._circ.center = self._pos
        
        
    def time_to_collision(self,other):
        """
        rdotr=np.dot(self._pos - other._pos,self._pos - other._pos)
        a=np.dot(self._vel-other._vel,self._vel-other._vel)
        b=2*np.dot(self._pos - other._pos,self._vel-other._vel)
        c=rdotr-(self._radius+other._radius)**2
        disc = b**2 - 4*a*c
        big = 10**3
        if disc < np.finfo(float).eps: #is disc is less than zero then no answers
            return big
        
        else:
            TTC1, TTC2 = (-b+np.sqrt(b**2-4*a*c))/2*a, (-b-np.sqrt(b**2-4*a*c))/2*a
            print('TTC1,2 are',TTC1,TTC2)
            
            if self._container is True:
                return TTC2
            
            else:
                return TTC1
        """
        vdotr=np.dot(self._pos - other._pos,self._vel-other._vel)
        vdotv=np.dot(self._vel-other._vel,self._vel-other._vel)
        rdotr=np.dot(self._pos - other._pos,self._pos - other._pos)
        Rsquared=(self._radius+other._radius)**2  # minus sign because colliding ball and container
        discprime = (vdotr)**2-(vdotv)*(rdotr-Rsquared)
        big = 10**3
        if discprime < np.finfo(float).eps:   #compare discrimanant to smallest floating point error, eg if disc is less than 0
            return big
        
        else:
            TTC1, TTC2 = (-vdotr+np.sqrt(discprime))/vdotv, (-vdotr-np.sqrt(discprime))/vdotv
            
            # One positive, one negative
            if TTC1 < np.finfo(float).eps and TTC2 > np.finfo(float).eps:
                return TTC2
                   
            if TTC1 > np.finfo(float).eps and TTC2 < np.finfo(float).eps:
                return TTC1
        
           # Both positive
            if TTC1 > np.finfo(float).eps and TTC2 > np.finfo(float).eps:
                if TTC1 < TTC2:
                    return TTC1
                else:
                    return TTC2
            '''
            Forgot to account for the following scenrario so code used to 
            stall as the dt list had a None floating in there, min() function
            can't operate on None as it uses < and > operators
            else function below was just to check that it was the 
            time_to_collision function generating the Nones
            '''
           
            if TTC1 < np.finfo(float).eps and TTC2 < np.finfo(float).eps:
                return big
#            else:
#                return big
    def collide(self,other):
        v1=self._vel
        v2=other._vel
        relvel= v1 - v2 #v=v1-v2
        relpos=self._pos-other._pos #r=r1-r2
        vdotr=np.dot(relvel,relpos)
        m1 = self._mass
        m2 = other._mass
        magsquaredpos = np.dot(relpos,relpos) #finds the magnitude squared of the relative postition
        
        v1prime = v1 - (2*m2/(m1+m2))*(vdotr/magsquaredpos)*relpos
        v2prime = v2 - (2*m1/(m1+m2))*(vdotr/magsquaredpos)*(-relpos) #(-v)dot(-r) equals vdotr
#        print(type(v2))
#        print(magsquaredpos)
        self._vel=v1prime
        other._vel=v2prime


class Container(Ball):
    def __init__(self,mass=99999, radius=-20,container=True):
        Ball.__init__(self,mass,radius,np.array([0.0,0.0]),np.array([0.0,0.0]),container)

'''
In between making the container &ball classes, a container and ball should be made,
this is what the following class simulation refers to, this is why it can use the 
methods defined above in ball
'''

class Simulation:
    def __init__(self, cont=Container(),num_frames=100, N=1):
        self._balls=[]    #a list of objects of class ball
        self._container=cont
        self.generate_balls(N)
        self._Nballs = N
        self._distances = []
        self._num_frames = num_frames
    
    def generate_balls(self,N,ballrad=1):
        spawnradius = abs(self._container._radius)-ballrad #minus the radius of the ball
        '''Generating radom coordinates within radius
        leng is range of inscribed square, biggest quare that will fit
        inside of circle radius R'''
        leng= spawnradius/np.sqrt(2) 
        coords=[]   #start adding balls from top left
        ranvels=[]  #list of random velocities
        balldiam = 2*ballrad
        buffer = 1.5                        #buffer of half a ball diameter between balls
        Nfit = int(2*leng/(balldiam*buffer)) #Integer Number of balls that fit in the length of the inscribed square
        step = balldiam*buffer
        if N > Nfit**2:
            raise Exception('Too many balls for specified radius')
        
        for h in range(Nfit):
            for w in range(Nfit):
                coords.append([-leng+0.01+w*step,leng-0.01-h*step])
        
        for v in range(N):
            ranvels.append([np.random.uniform(-2,2),np.random.uniform(-2,2)])
#        print('Nfit',Nfit, leng, spawnradius) #bug fixing, leng shouldnt be negative
#        print(coords)   
        pos = [[-1,0],[2,0.1],[0,-4],[4,-5],[3,-3]]
        vel = [[1,0],[-1,0],[0,-1],[0.5,0.2],[0.5,0.2]]
        
        xvals = []
        yvals = []
        for coord in coords:
            xvals.append(coord[0])
            yvals.append(coord[1])
            
#        print('xvals',xvals)
#        print('yvals',yvals)
        
        for i in range(N):
            b = Ball(1,ballrad,coords[i], ranvels[i])
            self._balls.append(b) 
        self._balls.append(self._container)
    
    def next_collision(self):#,other):
        '''
        next_collision method works by creating a list of dts,
        finding the smallest dt
        moving every ball by this mintime
        finding the two balls that do collide by creating another
        list of pairs (trace) of balls that collide for every dt
        finding the position of mintime in the list dt
        and looking for this position in the trace list
        and performing the collide function on these two balls
        note: !Container is ball 0!
        '''
        dt=[]
        trace=[]
        positions=[]   #will be filled with the position of every ball after every frame
        for a in range(0,self._Nballs):
            dt.append(self._balls[a].time_to_collision(self._container))
            trace.append([a,-1])   
            for b in range(a+1,self._Nballs):
               dt.append(self._balls[a].time_to_collision(self._balls[b])) #need to make a tracing list in here to track every pair of balls that have that dt
               trace.append([a,b])
        
        for g in range(0,self._Nballs):
            self._distances.append(np.linalg.norm(self._balls[g]._pos))
        
        
            
            
        
#        print('my trace',trace)
#        print('TEST',self._balls[0].time_to_collision(self._balls[2]))
#        print('dt',dt)
        
        mintime = min(dt)
        L=dt.index(mintime)
#        if mintime < 10**-4:
#            dt[L]=10**4
#            mintime = min(dt)
#            L=dt.index(mintime)
#        print('Lowest time position',L)
        print('Combination of balls',trace[L],'Lowest time dt:',dt[L])
        
        for a in range(self._Nballs):
            self._balls[a].move(mintime - 0.001)
            
        #other._ball.move(dt)
        self._balls[trace[L][0]].collide(self._balls[trace[L][1]])
        
        '''
        for trace[L][0,1]-1 the minus 1 is because the balls are indexed from 
        0 to N-1, and the trace index I made uses numbers as balls and 0 as the
        container. If trace[L][0,1] itself equals 0 then the container is
        colliding, it will look for the minus 1 ball in the list self._balls
        since the container is appended onto the list of balls in line 196 it is 
        the last element in the ball list and -1 index will find the container 
        '''
        
    def run(self, animate=False):
        if animate:
            f = pl.figure()
            l=1.3*abs(self._container._radius)
            ax = pl.axes(xlim=(-l, l), ylim=(-l, l))
            ax.add_artist(self._container.get_patch())
            for i in range(self._Nballs):
                ax.add_patch(self._balls[i].get_patch())
        for frame in range(self._num_frames):
            pl.title("Roberto's Balls")
            self.next_collision()
            if animate:
                pl.pause(0.005)
        if animate:
            pl.show()    
                   
    def plothist(self,ShowPlot=True):
        myhistogram=pl.hist(self._distances, bins=self._Nballs*self._num_frames
        if ShowPlot is True:
            print(myhistogram)
        

        
        
        
        
'''
        print('discprime',discprime) 
        print("vdotv",vdotv)
        print("vdotr",vdotr)
        print("rdotr",rdotr)
        print(Rsquared)
        print(TTC1, TTC2)
'''        
#B=Ball()
#B.vel()
#B.move(2.4)
#print(B._pos)