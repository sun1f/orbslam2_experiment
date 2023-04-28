# 画3D散点图
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

data1 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_before_opt.txt", usecols=(1, 2, 3))
x1 = data1[:, 0]
y1 = data1[:, 1]
z1 = data1[:, 2]
data2 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_after_DL.txt", usecols=(1, 2, 3))
x2 = data2[:, 0]
y2 = data2[:, 1]
z2 = data2[:, 2]
data3 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_after_LM.txt", usecols=(1, 2, 3))
x3 = data3[:, 0]
y3 = data3[:, 1]
z3 = data3[:, 2]

# 方式1：设置三维图形模式
fig = plt.figure()  # 创建一个画布figure，然后在这个画布上加各种元素。
ax = Axes3D(fig)  # 将画布作用于 Axes3D 对象上。

ax.scatter(x1, y1, z1)  # 画出(x1,y1,z1)的散点图。
ax.scatter(x2, y2, z2, c='r', marker='^')
ax.scatter(x3, y3, z3, c='g', marker='*')

ax.set_xlabel('X label')  # 画出坐标轴
ax.set_ylabel('Y label')
ax.set_zlabel('Z label')

plt.show()
