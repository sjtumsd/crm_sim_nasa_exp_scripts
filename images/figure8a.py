import math
import numpy
from math import atan
import matplotlib
import matplotlib.pyplot as plt

line_style=['--','-.',':','--','-.',':','--','-.',':']
marker_s=['o','D','s','^','v','x','D','p','1','<','>','d']
line_color=['b','g','r','c','m','y','k','b','g','r','c','m']
line_label=[]
title_name=r'Full VIPER Rover, Earth, m=88kg, $\omega=0.8$rad/s, GRC-1'

fig_name="figure/figure8a.png"

plt.figure(figsize = [10, 6])
font = {'weight': 'bold', 'size': 14}
plt.rc('font', **font)

angle = [0, 5, 10, 15, 20, 25, 30]

# our results
for i in range(7):
    file=open("../demos/viper_real_slope/outputs/GRC1/Earth/omega0.8/soil3" + "_slope" + str(i*5) + "deg" + "/body_position.txt","r")
    Time=[]
    vel=[]
    num=0
    for line in file:
        result=list(map(float, line.split()))
        
        Time.append(result[0])
        x=result[4]

        vel.append(x)
        num += 1
    file.close()

    plt.plot(Time,vel, label=""+str(angle[i])+r"$^o$",linewidth=2)
    #plt.plot(slip,slope,linestyle=line_style[k],marker=marker_s[k],label=line_label[k],linewidth=3,markersize=14)
plt.grid(linestyle='--')
plt.legend(loc='upper left',fontsize=12,ncol=7)

ax1=plt.gca()
# ax1.set_title(title_name,fontsize=14,weight='bold')
ax1.set_xlabel('Time (s)',fontsize=14,weight='bold')
ax1.set_ylabel('Velocity (m/s)',fontsize=14,weight='bold')
ax1.set_xlim([0,20.0])
ax1.set_ylim([-0.0, 0.25])
ax1.tick_params(axis='both',direction='in')

for axis in ['top','bottom','left','right']:
    ax1.spines[axis].set_linewidth(2)
ax1.tick_params(width = 2)

plt.savefig(fig_name, dpi=300)
