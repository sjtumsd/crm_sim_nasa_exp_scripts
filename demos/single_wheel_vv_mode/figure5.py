import math
import numpy
from math import atan
import matplotlib
import matplotlib.pyplot as plt
line_style=['--','-.',':','--','-.',':','--','-.',':']
marker_s=['o','D','s','^','v','x','D','p','1','<','>','d']
line_color=['b','g','r','c','m','y','k','b','g','r','c','m']
line_label=[]
title_name='Single wheel test under VV mode, Earth, m=17.5kg, v=0.2m/s, GRC-3'

# our results
for i in range(9):
    plt.clf()
    plt.figure(figsize = [10, 6])
    font = {'weight': 'bold', 'size': 20}
    plt.rc('font', **font)

    file=open("outputs/GRC3/Earth/soil2_slip"+str(i)+"/results.txt","r")
    Time=[]
    dbp=[]
    num=0
    for line in file:
        result=list(map(float, line.split()))
        
        Time.append(result[0])
        x=result[10]

        dbp.append(x)
        num += 1
    file.close()

    plt.plot(Time, dbp, label="slip = 0."+str(i), linewidth=2)

    plt.grid(linestyle='--')
    plt.legend(loc='upper right',fontsize=20,ncol=1)

    ax1=plt.gca()
    # ax1.set_title(title_name,fontsize=20,weight='bold')
    ax1.set_xlabel('Time (s)',fontsize=20,weight='bold')
    ax1.set_ylabel('DBP (N)',fontsize=20,weight='bold')
    ax1.set_xlim([0,20.0])
    ax1.set_ylim([-50, 200])
    ax1.tick_params(axis='both',direction='in')

    for axis in ['top','bottom','left','right']:
        ax1.spines[axis].set_linewidth(2)
    ax1.tick_params(width = 2)

    plt.savefig("./17_5kg_VV_Mode_GRC3_DBP_timeHis_" + "Slip0_"+str(i) + ".png", dpi=100)
