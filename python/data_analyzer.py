import numpy as np
import matplotlib.pyplot as plt
import pdb

# data = np.array([[ 0.05, 0.0000, 0.0001 ],
# [ 0.10, 0.0176, 0.0171 ],
# [ 0.15, 0.0412, 0.0398 ],
# [ 0.20, 0.0633, 0.0601 ],
# [ 0.25, 0.0833, 0.0823 ],
# [ 0.30, 0.1033, 0.1026 ],
# [ 0.35, 0.1236, 0.1236 ],
# [ 0.40, 0.1440, 0.1446 ],
# [ 0.45, 0.1650, 0.1649 ],
# [ 0.50, 0.1847, 0.1850 ],
# [ 0.55, 0.2059, 0.2050 ],
# [ 0.60, 0.2273, 0.2255 ],
# [ 0.65, 0.2482, 0.2451 ],
# [ 0.70, 0.2692, 0.2646 ],
# [ 0.75, 0.2891, 0.2845 ],
# [ 0.80, 0.3109, 0.3041 ],
# [ 0.85, 0.3318, 0.3234 ],
# [ 0.90, 0.3547, 0.3439 ],
# [ 0.95, 0.3813, 0.3693 ],
# [ 1.00, 0.3974, 0.3835 ]])

#data = np.loadtxt("output.txt", delimiter=',')
#data = np.loadtxt("../log/backward_calibration_LiDAR.txt", delimiter=',')
data = np.loadtxt("../data/motion_controller_v_w.txt")
#pdb.set_trace()


# [ml, bl] = np.polyfit(data[:,0],data[:,1],1)
# [mr, br] = np.polyfit(data[:,0],data[:,2],1)

# print("left motor: %f x + %f" % (ml, bl))
# print("right motor: %f x + %f" % (mr, br))
 
#plt.plot(data[:,0], data[:,1])
plt.plot(data[:,0], data[:,2])
# plt.plot(data[:,0], ml*data[:,0] + bl, 'r')
# plt.plot(data[:,0], mr*data[:,0] + br, 'g')
plt.legend()
plt.xlabel('Seconds', fontsize=16)
plt.ylabel('rad/s', fontsize=16)
plt.title("Robot's Rotational Velocity vs Time ")
plt.show()