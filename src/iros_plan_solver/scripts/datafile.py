import numpy as np
import matplotlib.pyplot as plt 
import os
import tempfile
import subprocess



def plotmakespantestresults():

    # pp =21 ; r increase ; scale = 50
    listMMppMS = np.array([[0.01,0.01,0.05,0.14,0.25,0.41,0.81,1.18,1.72,2.49,3.27,6.74,12.64,11.92,29.97,37.29,49.24,16.84,66.14],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
    listMMppMT = np.array([[274.001,465.901,759.011,686.428,707.243,1137.870,1456.660,1709.413,2122.756,2317.582,2369.856,2456.268,2413.026,2370.740,2865.454,3096.338,3201.052,3260.168,2924.061,3517.098],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
    listheteroppMS = np.array([[0.00,0.00,0.02,0.03,0.06,0.10,0.16,0.28,0.51,0.82,1.40,1.88,3.10,4.17,4.78,6.69,9.42,11.81,16.19,21.89],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
    listheteroppMT = np.array([[274.001,778.015,800.845,1156.924,1362.927,1765.557,2143.389,2173.390,2481.515,2261.193,2967.632,3068.954,3244.433,3356.905,3028.113,3165.452,3740.375,3692.133,3568.434,3910.180],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])

    # pp =21 ; r increase ; scale = 50
    listMMrMS = np.array([[64.22,75.53,100.46,256.05,369.55,295.25,197.85,294.00,476.30,979.05,802.66,546.26,678.48,760.73,0,0,0,0,0,0],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
    listMMrMT = np.array([[2997.954,2165.804,1895.840,1408.024,1305.849,1483.900,1347.301,1002.594,923.638,1088.770,1099.674,752.974,753.120,694.991,0,0,0,0,0,0],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
    listheterorMS = np.array([[26.44,24.26,30.46,21.94,28.30,32.61,57.59,69.19,82.70,62.42,65.66,73.40,84.08,83.46,92.10,108.13,116.25,121.70,142.97,141.02],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
    listheterorMT = np.array([[4029.385,4029.385,2437.922,2437.922,1965.476,1965.476,2299.395,2299.395,1987.608,1348.489,1216.177,1045.610,1178.861,1178.861,921.990,921.990,847.892,847.892,670.168,670.168],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])

    # pp =21 ; r =4 ; scale increase
    listMMbigMS = np.array([[113.58,35.74,101.71,31.96,171.57,90.98,54.44,105.57,52.92,307.50,122.77,71.48,134.37,55.25,307.64,238.87,128.40,269.86,163.33,148.44],[100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1050,1100]])
    listMMbigMT = np.array([[4091.945,3934.967,4548.272,6208.435,5943.240,6570.961,7614.650,6955.647,8405.132,9018.934,7340.208,9841.992,9257.008,8134.264,10395.471,13277.286,9279.090,13644.299,12629.317,14712.354],[100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1050,1100]])
    listheterobigMS = np.array([[12.72,21.09,22.64,13.41,21.62,19.87,18.25,15.02,17.95,29.33,20.63,16.55,23.84,21.44,27.66,22.06,17.48,18.80,16.94,29.83],[100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1050,1100]])
    listheterobigMT = np.array([[6747.832,7930.002,12934.316,13253.833,18998.846,13114.623,16666.548,16642.413,20143.914,26088.652,25864.753,26410.503,34011.918,25971.129,38722.927,45125.810,47278.296,43313.204,44576.282,53868.884],[100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1050,1100]])

    plt.subplot(2, 1, 1)
    plt.plot(listMMppMS[1][0:15],listMMppMS[0][0:15],marker = 'x',label='SwitchEnabled')
    plt.plot(listheteroppMS[1][0:15],listheteroppMS[0][0:15],marker = 'x', label='NoSwitch')
    # plt.title("Comparison problem resolution by popf w/ and w/o switch, fixed number of robots = 4, rising number of objectives")
    plt.ylabel("Time to find the solution (s)")

    plt.subplot(2, 1, 2)
    plt.plot(listMMppMT[1][0:15],listMMppMT[0][0:15],marker = 'x',label='SwitchEnabled')
    plt.plot(listheteroppMT[1][0:15],listheteroppMT[0][0:15],marker = 'x', label='NoSwitch')
    plt.xlabel("Increasing number of pollution points in small site area")
    plt.ylabel("Solution makespan total time (s)")
    plt.legend()
    plt.show()

    plt.subplot(2, 1, 1)
    plt.plot(listMMrMS[1][0:14],listMMrMS[0][0:14],marker = 'x',label='SwitchEnabled')
    plt.plot(listheterorMS[1][0:14],listheterorMS[0][0:14],marker = 'x', label='NoSwitch')
    # plt.title("Comparison problem resolution by popf w/ and w/o switch, fixed number of objectives = 21, rising number of robots")
    plt.ylabel("Time to find the solution (s)")
    

    plt.subplot(2, 1, 2)
    plt.plot(listMMrMT[1][0:15],listMMrMT[0][0:15],marker = 'x',label='SwitchEnabled')
    plt.plot(listheterorMT[1][0:15],listheterorMT[0][0:15],marker = 'x', label='NoSwitch')
    plt.xlabel("Increasing number of robots in small site area")
    plt.ylabel("Solution makespan total time (s)")
    plt.legend()
    plt.show()

    plt.subplot(2, 1, 1)
    plt.plot(listMMbigMS[1][0:15],listMMbigMS[0][0:15],marker = 'x',label='SwitchEnabled')
    plt.plot(listheterobigMS[1][0:15],listheterobigMS[0][0:15],marker = 'x', label='NoSwitch')
    # plt.title("Comparison problem resolution by popf w/ and w/o switch, fixed number of robots and objectives, rising problem area size")
    plt.ylabel("Time to find the solution (s)")
    
    plt.subplot(2, 1, 2)
    plt.plot(listMMbigMT[1][0:15],listMMbigMT[0][0:15],marker = 'x',label='SwitchEnabled')
    plt.plot(listheterobigMT[1][0:15],listheterobigMT[0][0:15],marker = 'x', label='NoSwitch')
    plt.xlabel("Different variations of problem pp = 21 and R = 4 with larger site area")
    plt.ylabel("Solution makespan total time (s)")
    plt.legend()
    plt.show()


# def plotcomplexitytestresult():
#     pp increase r =4
#     listMMppMS = np.array([[0.00,0.00,0.01,0.02,0.04,0.06,0.09,0.16,0.34,0.39,0.54,0.77,0.98,1.51,2.02,2.74,4.16,147.19,225.92,164.25],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
#     listMMppMT = np.array([[194.401,496.752,512.849,570.081,795.495,795.495,1019.513,1194.358,1141.928,1062.300,1087.805,1134.169,1237.185,1248.383,1528.722,1566.094,1636.983,1136.167,1581.838,1363.491],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
#     listheteroppMS = np.array([[0.00,0.00,0.01,0.02,0.04,0.05,0.09,0.16,0.35,0.38,0.50,0.77,0.98,1.47,1.98,2.86,3.96,5.40,7.02,7.97],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])
#     listheteroppMT = np.array([[194.401,496.752,512.849,570.081,795.495,795.495,1019.513,1194.358,1141.928,1062.300,1087.805,1134.169,1237.185,1248.383,1528.722,1566.094,1636.983,1636.983,1749.623,1731.777],[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]])

#     pp =21 r increase
#     listMMrMS = np.array([[],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
#     listMMrMT = np.array([[],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
#     listheterorMS = np.array([[],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])
#     listheterorMT = np.array([[],[4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]])

#     pp = 21 r = 8, bigger area,
#     listMMbigMS = np.array([[72.00,83.26,63.76,113.76,189.64,444.24,264.02,82.18,159.50,169.57,241.12,120.82,93.66,118.43,103.51,128.46,201.14,175.07,146.12,97.71],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]])
#     listMMbigMT = np.array([[2446.147,3070.485,2693.145,2581.274,2424.565,3282.880,1971.929,2336.977,2675.069,3673.390,2804.050,2932.943,2582.650,1876.842,2285.995,2932.327,2752.704,2831.868,2243.115,3312.796],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]])
#     listheterobigMS = np.array([[26.00,24.76,24.86,33.15,26.43,25.61,27.70,24.02,33.29,27.29,35.93,28.53,29.46,26.39,26.50,24.73,25.58,21.75,28.51,26.15],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]])
#     listheterobigMT = np.array([[3233.941,4212.451,4317.543,4682.777,3285.539,3699.115,3179.276,3496.965,2791.143,4532.124,4149.415,5024.040,3559.761,3197.361,3360.332,3877.010,3708.136,3744.331,3140.450,3687.979],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]])


#     xaxisppmm=[1,2,3,4,5,6,7,8,10,11,12,14,15]
#     xaxispphe=[4]

#     xaxisrmm=[1,2,3,4,5,6,7,8,10,11,12,14,15]
#     xaxisrhe=[1,2,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20]

#     plt.subplot(1, 2, 1)
#     plt.ylabel("Time to find the solution (s)")
#     plt.xlabel("Number of pollution points")
#     plt.plot( xaxisppmm,listMMpp[0],marker = 'o',label='SwitchEnabled')
#     plt.plot( xaxispphe,listheteropp[0],marker = 'x', label='NoSwitch')

#     plt.subplot(2, 1, 2)
#     plt.ylabel("Solution makespan total time (s)")
#     plt.xlabel("Number of pollution points")
#     plt.plot( xaxisppmm,listMMpp[1],marker = 'o',label='SwitchEnabled')
#     plt.plot( xaxispphe,listheteropp[1],marker = 'x', label='NoSwitch')

#     plt.subplot(1, 2, 2)
#     plt.ylabel("Time to find the solution (s)")
#     plt.xlabel("Number of robots")
#     plt.plot(xaxisrmm, listMMr[0],marker = 'o',label='SwitchEnabled')
#     plt.plot(xaxisrhe,listheteror[0],marker = 'x', label='NoSwitch')
    
#     plt.subplot(2, 2, 2)
#     plt.ylabel("Solution makespan total time (s)")
#     plt.xlabel("Number of robots")
#     plt.plot(xaxisrmm, listMMr[1],marker = 'o',label='SwitchEnabled')
#     plt.plot(xaxisrhe,listheteror[1],marker = 'x', label='NoSwitch')

#     plt.title("Comparison problem resolution by popf w/ and w/o switch standart: 2r and 21 pp")
#     plt.legend()

#     plt.show()




# plotmakespantestresults()

# plotcomplexitytestresult()

# study = "r"
# domain = "switch"
# for batch in range(0,10):
#     for num in range(0,14):
#         os.system("echo {0}".format("increase"+ study +"/batch" + str(batch) + "/problem" + str(num)))
#         os.system("ros2 run popf popf Ros2PLAN_ws/src/domainfiles/MMdomain.pddl Ros2PLAN_ws/src/Compareproblems/{0}.pddl > ProblemOutput/{1}.txt".format("increase"+ study +"/batch" + str(batch) + "/problem" + str(num),"increase"+ study +"/batch" + str(batch) + "/problem" + domain + str(num)))
#         os.system("echo \n")
