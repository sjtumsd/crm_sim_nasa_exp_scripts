import math
import numpy
from math import atan
import matplotlib
import matplotlib.pyplot as plt
import csv

line_style=['--','-.',':',':','--','-.','-',':','--','-.','-',':']
marker_s=['o','D','s','^','v','p','D','p','1','<','>','d']
line_color=['b','g','c','c','m','y','k','b','g','r','c','m']

result_name=["../demos/viper_real_slope/outputs/GRC1/Earth/omega0.8/soil", 
             "../demos/viper_real_slope/outputs/GRC1/Earth/omega0.8/soil", 
             "../demos/viper_real_slope/outputs/GRC1/Earth/omega0.8/soil", 
             "../demos/viper_real_slope/outputs/GRC1/Moon/omega0.8/soil", 
             "../demos/viper_real_slope/outputs/GRC1/Moon/omega0.8/soil", 
             "../demos/viper_real_slope/outputs/GRC1/Moon/omega0.8/soil"]
line_label=[r'Earth, $\varphi=33.4,\rho=1660$', 
            r'Earth, $\varphi=37.2,\rho=1730$', 
            r'Earth, $\varphi=38.4,\rho=1760$',
            r'Moon, $\varphi=33.4,\rho=1660$', 
            r'Moon, $\varphi=37.2,\rho=1730$', 
            r'Moon, $\varphi=38.4,\rho=1760$']
title_name='FullRover, Earth/Moon Vs. Experiments, GRC1'
fig_name = "figure/figure11d.png"

plt.figure(figsize = [10, 6])
font = {'weight': 'bold', 'size': 14}
plt.rc('font', **font)

# TREC Nominal Tests, GRC3
experimental1=numpy.array([
-0.20591778409882977, -1.6592712566932661,
4.9194457558859455, 8.099645658773934,
9.798837008076429, 13.81841734306245,
9.88365040348528, 16.932806217856843,
20.02846154750844, 20.120785162360185,
30.008081466559357, 19.857759468000914,
30.011979388059924, 22.298832807729973,
39.990792840593485, 21.530757456875037,
40.15584965447951, 24.897587152988372,
40.073186836450276, 23.129997362182422,
50.13506634027175, 24.382036630381265,
60.117240069960985, 25.718334848258273,
60.11092274890834, 21.762112539042214,
60.03726547365628, 25.63424391243789,
70.09618793358077, 25.034434440152616,
70.10371495440944, 29.748231234112183,
80.16613210257583, 31.336970273308133,
80.08992101668545, 33.60977773446753])

# MGRU3 2021 SLOPElab Tests, GRC1
experimental2=numpy.array([
3.1504614500604013, 0.27322413552669644,
5.154396334609679, 5.237445584511807,
13.465437029246175, 10.026680600615933,
43.41343674026234, 14.961499624489019,
75.59749928174078, 20.23066622535026,
88.45956494491666, 25.09928778925685])

# MGRU3 2022 SLOPElab Tests, GRC1
experimental3=numpy.array([
2.511739968312586, 0.2738961909578279,
5.075093793736109, 5.5742293624378085,
10.431509990944054, 10.029872863913809,
35.50979604797804, 15.30651608144639,
73.8406119736756, 19.97998954953804,
83.82896861333123, 25.188335133881836,
89.6652323883674, 30.148524250280154])

# p1,=plt.plot(experimental1[::2]/100,experimental1[1::2],color='k',linestyle='--',marker='*',markersize=16,linewidth=3)
# p2,=plt.plot(experimental2[::2]/100,experimental2[1::2],color='k',linestyle='-.',marker='<',markersize=14,linewidth=3)
# p3,=plt.plot(experimental3[::2]/100,experimental3[1::2],color='k',linestyle=':', marker='*',markersize=16,linewidth=3)
# l1=plt.legend([p1,p2,p3],["TREC Nominal Tests, GRC3","MGRU3 2021 SLOPElab Tests, GRC1","MGRU3 2022 SLOPElab Tests, GRC1"],
#     loc='lower right',fontsize=14)
# l1=plt.legend([p1],["TREC Nominal Tests, GRC3"], loc='lower right',fontsize=14)

# our results
angW = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
i_range = [7,7,7,7,7,7]
for n in range(2):
    for k in range(3):
        slip=[]
        power=[]
        for i in range(7):
            base_dir = result_name[3*n+k] + str(k+1) + "_slope" + str(i*5) + "deg"
            file=open(base_dir + "/body_position.txt","r")
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

            sumsum=0.0
            numnum=0
            for j in range(num//2,num-1):
                sumsum+=vel[j]
                numnum+=1

            slip.append(1 - sumsum/numnum/angW[3*n+k]/0.25)

            total_torque = 0.0
            for n_wheel in range(4):
                tor=[]
                num=0
                with open(base_dir+"/fsi/"+'FSI_body'+str(n_wheel)+'.csv') as csvfile:
                    csvReader = csv.reader(csvfile, delimiter=',')
                    for row in csvReader:
                        if num==0:
                            num += 1
                            continue
                        tor.append(-float(row[15]))
                        num += 1
                sumsum=0.0
                numnum=0
                for j in range(num//2,num-1):
                    sumsum+=tor[j]
                    numnum+=1
                total_torque = total_torque + sumsum/numnum

            if n < 1:
                power.append(total_torque * angW[3*n+k] / (9.81*math.sqrt(9.81)))
            else:
                power.append(total_torque * angW[3*n+k] / (1.62*math.sqrt(1.62)))

        plt.plot(slip,power,linestyle=line_style[3*n+k],marker=marker_s[3*n+k],markersize=14,label=line_label[3*n+k],linewidth=3)

plt.grid(linestyle='--')
l2=plt.legend(loc='upper left',fontsize=14,ncol=2)

# plt.gca().add_artist(l1)

ax1=plt.gca()
# ax1.set_title(title_name,fontsize=14,weight='bold')
ax1.set_xlabel('Slip',fontsize=14,weight='bold')
ax1.set_ylabel('Scaled power',fontsize=14,weight='bold')
ax1.set_xlim([-0.1,1.0])
ax1.set_ylim([0.0,16])
ax1.tick_params(axis='both',direction='in')

for axis in ['top','bottom','left','right']:
    ax1.spines[axis].set_linewidth(2)
ax1.tick_params(width = 2)

plt.savefig(fig_name, dpi=300)