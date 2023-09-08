# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 10:08:55 2019

Simulation object

@author: rak1618
"""

import pylab as pl
import numpy as np
import Classes as cl

#FS = cl.Simulation()
#
#FS._ball =  cl.Ball(radius=1,pos=[-2,0],vel=[1,1])
#
##FS._ball(pos=[-5,0],vel=[1,0]) 
#cl.Ball()

#dt = cl.Ball.time_to_collision(FS._ball,FS._container)
#FS._ball.move(dt)
#print(dt)
#newvel = cl.Ball.collide(FS._ball,FS._container)
#print(FS._ball.vel())




#b = cl.Ball(radius=1,pos=[-2,0],vel=[1,0.1])
#c = cl.Container()

def findKE(b):
    KE = 0.5*(b._mass)*(np.dot(b._vel,b._vel))
    return KE
    

#print(findKE(b),'before')

NO = cl.Simulation(cont=cl.Container(),num_frames=300,N=50)
NO.run(animate=False)
NO.plothist()

#%%
print('distances list',NO._distances)
print('distance list length', len(NO._distances))
#print(findKE(b),'after')
#NO.next_collision()
#print(NO._ball.vel())
#print(NO._ball.pos(),'ball new pos')

#NO.next_collision()
#print(NO._ball.vel())
#print(NO._ball.pos(),'ball new pos2')



        
