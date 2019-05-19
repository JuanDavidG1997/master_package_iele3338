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

global X, Y, THETA

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



def kill_node():
	rospy.signal_shutdown('Quit')


def positionCallback(msg):
	global X, Y, THETA
	X = msg.position.x 
	Y = msg.position.y
	THETA = msg.orientation.w

def uncertaintyCallback(msg):
	pass

def main_master_plot():

	global X, Y, THETA

	rospy.loginfo("Starting master plot node")

	rospy.init_node('main_master_plot', anonymous=True)
	rospy.Subscriber('robot_position', Pose, positionCallback)
	rospy.Subscriber('robot_uncertainty', Covariance, uncertaintyCallback)
	
	
	rate = rospy.Rate(4)

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
		
		if addedImg:
			robotImgMPL.remove()
			del robotImgMPL

		robotImgMPL = ax1.imshow(img, zorder=10)

		tr = transforms.Affine2D().scale(0.3,0.3)
		tr.rotate_around(132*0.3/2, 132*0.3/2, THETA)
		tr.translate(X-132*0.3/2, Y-132*0.3/2)


		tr = tr + ax1.transData
		robotImgMPL.set_transform(tr)

		addedImg = True
		plt.draw()
		plt.pause(0.001)

		rate.sleep()


if __name__ == '__main__':
	try:
		main_master_plot()
	except rospy.ROSInterruptException:
		rospy.logfatal("Fatal error on node")
