import re
import sys
import numpy as np
import matplotlib.pyplot as plt


def CalAve(timeList):
	sum = 0
	out = 0
	for i in range(len(timeList)):
		# if(timeList[i]>32):
		# 	out=out+1
		# 	sum = sum+timeList[i]*10000
		# else:
		sum = sum+timeList[i]*10000/13
	average = sum/(len(timeList))
	print("Total samples:",len(timeList))
	print("out:",out)
	# print("sum:",sum)
	return average

def CalAve1(timeList):
	sum = 0
	out = 0
	for i in range(len(timeList)):
		sum = sum+timeList[i]*10000
	average = sum/(len(timeList))
	print("Total samples:",len(timeList))
	print("out:",out)
	# print("sum:",sum)
	return average

def CalHisto(timeList):
	his = [0 for n in range(26)]
	for i in range(len(timeList)):
		if(int(timeList[i]*5000)>25):
			his[25] = his[25]+1
		else:
			# print(i)
			his[int(timeList[i]*5000)] = his[int(timeList[i]*5000)]+1
	return his

timeList = []
delList = []
f = open(r"/home/xhu/Desktop/res/seq_thread_total.txt")
line = f.readline()
while line:
	timeres = re.findall(r"%&(.+?)&%",line)
	if(len(timeres)!=0):
		timeList.append(float(timeres[0]))
	line = f.readline()
# print(len(timeList))
timeList.sort()
# print(timeList[-18])
# timeList = timeList[:-6]


average = CalAve1(timeList)
print(average)

# for i in range(1,int(len(timeList)/100)):
# 	# ave = CalAve(timeList[:i])
# 	# if(timeList[i*100]*10000<50):
# 	plt.scatter(i, timeList[i*100]*10000, s=0.5, c='green')

timeList1 = []
f = open(r"/home/xhu/Desktop/res/orb_total.txt")
line = f.readline()
while line:
	timeres = re.findall(r"%&(.+?)&%",line)
	if(len(timeres)!=0):
		timeList1.append(float(timeres[0]))
	line = f.readline()
# print(len(timeList1))



average = CalAve1(timeList1)
print(average)

# for i in range(1,int(len(timeList1)/100)):
# 	# ave = CalAve(timeList[:i])
# 	# if(timeList1[i*200]*10000<50):
# 	plt.scatter(i, timeList1[i*100]*10000, s=0.5, c='red')

# plt.ylim(30,300)

tList = CalHisto(timeList)
print(tList)

name_list = ['0~2','2~4','4~6','6~8','8~10','10~12','12~14','14~16','16~18','18~20','20~22','22~24',\
			'24~26','26~28','28~30','30~32','32~34','34~36','36~38','38~40','40~42','42~44','44~46',\
			'46~48','48~50','>50']

x =list(range(len(tList)))
total_width, n = 0.5, 2
width = total_width / n

plt.bar(x, tList, width = width, color='red', label='Seq-Orb')

for i in range(len(x)):
    x[i] = x[i] + width

tList1 = CalHisto(timeList1)
print(tList1)

plt.bar(x, tList1, width = width, tick_label = name_list, color='green', label='ORB')

plt.xlabel('time usage per loop/0.01s')

plt.ylabel('Number of Samples')

plt.legend()

plt.show()

#include<time.h>
# vector<float> vTimesTrack;
# clock_t start,ends;