import numpy as np
import matplotlib.pyplot as plt


def drow_Ground_truth(path):
	pose = np.loadtxt(path)

	x = np.array([])
	y = np.array([])
	for num in range(len(pose)):
		x=np.append(x,pose[num][3])
		y=np.append(y,pose[num][11])
	plt.scatter(x, y, s=0.1) 
	plt.show()

def drow_result(path):
	pose = np.loadtxt(path)
	x = np.array([])
	y = np.array([])
	for num in range(len(pose)):
		x=np.append(x,pose[num][1])
		y=np.append(y,pose[num][3])
	plt.scatter(x, y, s=0.1, c='red') 
	plt.show()

def drow_compare(g_path,r_path):
	pose = np.loadtxt(g_path)
	# np.set_printoptions(suppress=True)
	# print(pose)

	g_x = np.array([])
	g_y = np.array([])
	for num in range(len(pose)):
		g_x=np.append(g_x,pose[num][3])
		g_y=np.append(g_y,pose[num][11])

	pose = np.loadtxt(r_path)
	r_x = np.array([])
	r_y = np.array([])
	for num in range(len(pose)):
		r_x=np.append(r_x, pose[num][1])
		r_y=np.append(r_y, pose[num][3])


	plt.scatter(g_x, g_y, s=0.1)
	plt.scatter(r_x, r_y, s=0.1, c='red')
	plt.show()


if __name__ == '__main__':
	g_path = '/home/xhu/Code/dataset/poses/00.txt'
	# drow_Ground_truth(g_path)
	r_path = '/home/xhu/Code/ORB_SLAM2/Examples/Monocular/KeyFrameTrajectory.txt'
	drow_compare(g_path,r_path)