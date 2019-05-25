#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from hivemind.srv import plotting
import rospy

data={}
colors = ["#15A186", "#F39C11", "#27AE61", "#D25400", "#2A80B9", "#C1392B", "#8F44AD", "#2D3E50",
            "#1BBC9B", "#F1C40F", "#2DCC70", "#E77E23", "#3598DB", "#E84C3D", "#9B58B5"]
#Colors for graphing plots


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def handle_plotting(req):
    if req.id not in data:
        data[req.id] = [req.x], [req.y], [req.z]
    else:
        data[req.id][0].append(req.x)
        data[req.id][1].append(req.y)
        data[req.id][2].append(req.z)

    return []


def plotting_server():
    rospy.init_node('plotting_server')
    s = rospy.Service('plotting', plotting, handle_plotting)
    while True:
        for id in data:
            #print("id ",id)
            ax.plot(data[id][0], data[id][1], data[id][2], color = colors[id%len(colors)])
        plt.pause(0.0000000001)
    rospy.spin()

if __name__ == "__main__":
    plotting_server()
    plt.show()
