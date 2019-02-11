import numpy as np
import matplotlib.pyplot as plt


def gen_data(path):
	data = np.loadtxt(path)
	points = [[],[],[]]
	for num in range(len(data)):
		points[0].append(data[num][3])
		points[1].append(data[num][7])
		points[2].append(data[num][11])
	return points

def drow_compare(g_path,r_path):
	


	plt.scatter(g_path[0], g_path[2], s=0.5)
	plt.scatter(list(r_path[0]), list(r_path[2]), s=0.5, c='red')
	plt.show()



if __name__ == '__main__':
	g_path = '/home/xhu/Code/dataset/poses/00.txt'
	# drow_Ground_truth(g_path)
	r_path = '/home/xhu/Code/ORB_SLAM2/Examples/Stereo/CameraTrajectory.txt'

	drow_compare(gen_data(g_path) ,gen_data(r_path))