#!/usr/bin/env python
import rospy
import rospkg
from master_msgs_iele3338.msg import Covariance
from geometry_msgs.msg import Pose
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)
from matplotlib import transforms
from matplotlib.patches import Ellipse, Wedge
import numpy as np
import threading

global X, Y, THETA, trajectory_x, trajectory_y
global sigma_matrix

rospack = rospkg.RosPack()
plt.rcParams["figure.figsize"] = (7,6)
plt.rcParams["figure.subplot.left"] = 0
plt.rcParams["figure.subplot.right"] = 1
plt.rcParams["figure.subplot.top"] = 0.95
plt.rcParams["figure.subplot.bottom"] = 0.05
image=rospack.get_path('master_package_iele3338')+'/resources/robot.png'
img=plt.imread(image)
frame_height=40
X = 0
Y = 0
THETA = 0
trajectory_x = []
trajectory_y = []
sigma_matrix = np.matrix([ [0,0,0],[0,0,0],[0,0,0] ])
lock = threading.Lock()


def kill_node():
	rospy.signal_shutdown('Quit')


def positionCallback(msg):
	global X, Y, THETA, trajectory_x, trajectory_y

	lock.acquire()
	X = msg.position.x/10
	Y = msg.position.y/10
	THETA = msg.orientation.w
	trajectory_x.append(X)
	trajectory_y.append(Y)
	lock.release()

def uncertaintyCallback(msg):
	global sigma_matrix

	sigma_matrix = np.matrix([ [msg.sigma11, msg.sigma12, msg.sigma13], [msg.sigma21, msg.sigma22, msg.sigma23], [msg.sigma31, msg.sigma32, msg.sigma33] ])

def main_master_plot():

	global X, Y, THETA, trajectory_x, trajectory_y
	global sigma_matrix

	rospy.loginfo("Starting master plot node")

	rospy.init_node('main_master_plot', anonymous=True)
	rospy.Subscriber('robot_position', Pose, positionCallback)
	rospy.Subscriber('robot_uncertainty', Covariance, uncertaintyCallback)
	
	
	rate = rospy.Rate(30)

	if matplotlib.__version__ < '2.2.4':
		rospy.logfatal("Update matplotlib to run the plot node: Version 2.2.4 is required")
		kill_node()

	f, ax1 = plt.subplots()

	ax1.grid(b=True, which='major', color='#AAAAAA', linestyle='-')

	plt.xlim(0,250)
	plt.ylim(0,250)

	plt.ion()
	win = plt.gcf().canvas.manager.window
	win.protocol("WM_DELETE_WINDOW", kill_node)
	f.canvas.set_window_title('Robot position')
	size = f.get_size_inches()*f.dpi
	windowWidth = size[0]
	windowHeight = size[1]
	positionRight = int(win.winfo_screenwidth()/2 - windowWidth/2)
	positionDown = int(win.winfo_screenheight()/2.3 - windowHeight/2)
	win.geometry("+{}+{}".format(positionRight, positionDown))

	plt.show()
	addedImg = False

	while not rospy.is_shutdown():

		x_unc = np.sqrt(sigma_matrix[0,0]) / 10
		y_unc = np.sqrt(sigma_matrix[1,1]) / 10
		theta_unc = np.sqrt(sigma_matrix[2,2]) * 180/np.pi
		
		if addedImg:
			robotImgMPL.remove()
			trajectory.remove()
			oval_uncertainty.remove()
			wedge_uncertainty.remove()
			del robotImgMPL
			del trajectory
			del oval_uncertainty
			del wedge_uncertainty

		robotImgMPL = ax1.imshow(img, zorder=10)

		tr = transforms.Affine2D().scale(0.3,0.3)
		tr.rotate_around(132*0.3/2, 132*0.3/2, THETA)
		tr.translate(X-132*0.3/2, Y-132*0.3/2)

		tr = tr + ax1.transData
		robotImgMPL.set_transform(tr)

		lock.acquire()
		trajectory, = ax1.plot(trajectory_x, trajectory_y, c='firebrick')
		lock.release()

		oval_uncertainty = Ellipse((0,0), width=x_unc*2, height=y_unc*2, facecolor='lightblue', edgecolor='midnightblue', alpha=0.3, zorder=20)
		tr_oval = transforms.Affine2D().translate(X, Y) #rotate(THETA)
		oval_uncertainty.set_transform(tr_oval + ax1.transData)
		ax1.add_patch(oval_uncertainty)

		wedge_uncertainty = Wedge((0,0), 45, theta1=-theta_unc, theta2=theta_unc, zorder=5, color='cornflowerblue', alpha=0.5)
		tr_wedge = transforms.Affine2D().rotate(THETA).translate(X, Y)
		wedge_uncertainty.set_transform(tr_wedge + ax1.transData)
		ax1.add_patch(wedge_uncertainty)

		

		addedImg = True
		plt.draw()
		plt.pause(0.001)

		rate.sleep()



if __name__ == '__main__':
	try:
		main_master_plot()
	except rospy.ROSInterruptException:
		rospy.logfatal("Fatal error on node")
