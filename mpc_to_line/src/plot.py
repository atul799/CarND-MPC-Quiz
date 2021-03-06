# -*- coding: utf-8 -*-
"""
This script is a workaround as matplotlibcpp doesn't work in docker
plots cte/delta and velocity for MPC

@author: atpandey
"""

import matplotlib.pyplot as plt

 

#%%
cte_file='../outputs/mpc_cte.out'
cte_list=[]
#with open(cte_file, mode='rb') as f:
cte_list = [line.rstrip('\n') for line in open(cte_file)]
    #for line in f:
    #    cte_list.append(line)
    
#%%
delta_file='../outputs/mpc_delta.out'
delta_list=[]
delta_list = [line.rstrip('\n') for line in open(delta_file)]
    

#%%
velocity_file='../outputs/mpc_velocity.out'
velocity_list=[]
velocity_list = [line.rstrip('\n') for line in open(velocity_file)]
    
#%%

#fig, ax = plt.subplots(nrows=3, ncols=1)
nx=len(cte_list)
x=range(nx)
#print(nx)
xd=range(nx-1)

#%%
plt.rcParams['axes.grid'] = True
ax[0].plot(x,cte_list)
ax[0].set_title('CTE')
ax[1].plot(xd,delta_list)
ax[1].set_title('Delta (Radians)')
ax[2].plot(x,velocity_list)
ax[2].set_title('Velocity')

plt.savefig('output.png')

#%%
fig=plt.figure()
plt.rcParams['axes.grid'] = True

ax0 = fig.add_subplot(311)
ax1 = fig.add_subplot(312)
ax2 = fig.add_subplot(313)

ax0.set_title('CTE')
ax1.set_title('Delta (Radians)')
ax2.set_title('Velocity')

ax0.plot(x, cte_list, linewidth=3)
ax1.plot(xd,delta_list , linewidth=3)
ax2.plot(x,velocity_list , linewidth=3)
plt.savefig('../outputs/output.png')
           

