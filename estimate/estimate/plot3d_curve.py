# 画3D曲线图
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

# 设置图例字号
mpl.rcParams['legend.fontsize'] = 10

# 方式2：设置三维图形模式
fig = plt.figure()
ax = fig.gca(projection='3d')

# 测试数据
data1 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_before_opt.txt", usecols=(1, 2, 3))
x1 = data1[:, 0]
y1 = data1[:, 1]
z1 = data1[:, 2]
data2 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_after_GN.txt", usecols=(1, 2, 3))
x2 = data2[:, 0]
y2 = data2[:, 1]
z2 = data2[:, 2]
data3 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_after_LM.txt", usecols=(1, 2, 3))
x3 = data3[:, 0]
y3 = data3[:, 1]
z3 = data3[:, 2]
data4 = np.loadtxt(
    "/home/sun/slambook2/ch10/result/txt/3D_after_DL.txt", usecols=(1, 2, 3))
x4 = data4[:, 0]
y4 = data4[:, 1]
z4 = data4[:, 2]
# 绘制图形
ax.plot(x1, y1, z1, label='Origin', color='green', linestyle='--', alpha=0.5)
ax.plot(x2, y2, z2, label='GN', color='red', alpha=0.5)
ax.plot(x3, y3, z3, label='LM', color='blue', alpha=0.5)
ax.plot(x3, y3, z3, label='Dog-Leg', color='grey', alpha=0.5)

# 显示图例
ax.legend()

# 显示标题
plt.title("sphere2500")

# 显示图形
plt.show()
