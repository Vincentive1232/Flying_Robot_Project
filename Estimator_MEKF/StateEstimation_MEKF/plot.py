import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def average_every_ten(arr):
    # 确保数组的长度是10的倍数
    if len(arr) % 10 != 0:
        raise ValueError("数组长度必须是10的倍数")
    
    # 将数组重塑为 (len(arr) / 10, 10) 的形状
    reshaped = np.reshape(arr, (-1, 10))
    
    # 对每一行求平均值，得到一个新的数组
    averaged = np.mean(reshaped, axis=1)
    
    return averaged


def quaternion_to_rpy(qw, qx, qy, qz):
    # 计算 Roll, Pitch, Yaw 对应的数组
    roll = np.zeros_like(qw)
    pitch = np.zeros_like(qw)
    yaw = np.zeros_like(qw)
    
    for i in range(len(qw)):
        # 计算 Roll (X 轴旋转)
        sinr_cosp = 2.0 * (qw[i] * qx[i] + qy[i] * qz[i])
        cosr_cosp = 1.0 - 2.0 * (qx[i] ** 2 + qy[i] ** 2)
        roll[i] = math.atan2(sinr_cosp, cosr_cosp)

        # 计算 Pitch (Y 轴旋转)
        sinp = 2.0 * (qw[i] * qy[i] - qz[i] * qx[i])
        if sinp >= 1.0:
            pitch[i] = math.pi / 2  # π/2
        elif sinp <= -1.0:
            pitch[i] = -math.pi / 2  # -π/2
        else:
            pitch[i] = math.asin(sinp)

        # 计算 Yaw (Z 轴旋转)
        siny_cosp = 2.0 * (qw[i] * qz[i] + qx[i] * qy[i])
        cosy_cosp = 1.0 - 2.0 * (qy[i] ** 2 + qz[i] ** 2)
        yaw[i] = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



# 读取数据
with open('data/output.txt') as file:
    data = file.readlines()

with open('data/groundtruth.txt') as file:
    data_gt = file.readlines()

pos_x = np.array([float(data[i].split(", ")[0]) for i in range(len(data))])
pos_y = np.array([float(data[i].split(", ")[1]) for i in range(len(data))])
pos_z = np.array([float(data[i].split(", ")[2]) for i in range(len(data))])

vel_x = np.array([float(data[i].split(", ")[3]) for i in range(len(data))])
vel_y = np.array([float(data[i].split(", ")[4]) for i in range(len(data))])
vel_z = np.array([float(data[i].split(", ")[5]) for i in range(len(data))])

rot_w = np.array([float(data[i].split(", ")[6]) for i in range(len(data))])
rot_x = np.array([float(data[i].split(", ")[7]) for i in range(len(data))])
rot_y = np.array([float(data[i].split(", ")[8]) for i in range(len(data))])
rot_z = np.array([float(data[i].split(", ")[9]) for i in range(len(data))])

pos_x_r = np.array([float(data_gt[i].split(", ")[0]) for i in range(len(data_gt))])
pos_y_r = np.array([float(data_gt[i].split(", ")[1]) for i in range(len(data_gt))])
pos_z_r = np.array([float(data_gt[i].split(", ")[2]) for i in range(len(data_gt))])

vel_x_r = np.array([float(data_gt[i].split(", ")[3]) for i in range(len(data_gt))])
vel_y_r = np.array([float(data_gt[i].split(", ")[4]) for i in range(len(data_gt))])
vel_z_r = np.array([float(data_gt[i].split(", ")[5]) for i in range(len(data_gt))])

rot_w_r = np.array([float(data_gt[i].split(", ")[6]) for i in range(len(data_gt))])
rot_x_r = np.array([float(data_gt[i].split(", ")[7]) for i in range(len(data_gt))])
rot_y_r = np.array([float(data_gt[i].split(", ")[8]) for i in range(len(data_gt))])
rot_z_r = np.array([float(data_gt[i].split(", ")[9]) for i in range(len(data_gt))])

error_pos_x = pos_x - pos_x_r
error_pos_y = pos_y - pos_y_r
error_pos_z = pos_z - pos_z_r

error_vel_x = vel_x - vel_x_r
error_vel_y = vel_y - vel_y_r
error_vel_z = vel_z - vel_z_r

time = np.arange(0, len(data), 10)


# 设置子图布局
fig, axs = plt.subplots(4, 2, figsize=(15, 20))
fig.tight_layout(pad=4.0)

# 位置图
axs[0, 0].plot(pos_x, color="black", label="pos_x")
axs[0, 0].plot(pos_y, color="blue", label="pos_y")
axs[0, 0].plot(pos_z, color="green", label="pos_z")
axs[0, 0].plot(pos_x_r, color="black", label="pos_x_real", linestyle="dashed")
axs[0, 0].plot(pos_y_r, color="blue", label="pos_y_real", linestyle="dashed")
axs[0, 0].plot(pos_z_r, color="green", label="pos_z_real", linestyle="dashed")
axs[0, 0].legend()
axs[0, 0].set_title("Position")

# 速度图
axs[0, 1].plot(vel_x, color="black", label="vel_x")
axs[0, 1].plot(vel_y, color="blue", label="vel_y")
axs[0, 1].plot(vel_z, color="green", label="vel_z")
axs[0, 1].plot(vel_x_r, color="black", label="vel_x_real", linestyle="dashed")
axs[0, 1].plot(vel_y_r, color="blue", label="vel_y_real", linestyle="dashed")
axs[0, 1].plot(vel_z_r, color="green", label="vel_z_real", linestyle="dashed")
axs[0, 1].legend()
axs[0, 1].set_title("Velocity")

# 旋转图

axs[1, 0].plot(rot_w, color="yellow", label="qw")
axs[1, 0].plot(rot_x, color="red", label="qx")
axs[1, 0].plot(rot_y, color="green", label="qy")
axs[1, 0].plot(rot_z, color="blue", label="qz")
axs[1, 0].plot(rot_w_r, color="yellow", label="qw_real", linestyle="dashed")
axs[1, 0].plot(rot_x_r, color="red", label="qx_real", linestyle="dashed")
axs[1, 0].plot(rot_y_r, color="green", label="qy_real", linestyle="dashed")
axs[1, 0].plot(rot_z_r, color="blue", label="qz_real", linestyle="dashed")
axs[1, 0].legend()
axs[1, 0].set_title("Rotation")



# rpy图
row, pitch, yaw = quaternion_to_rpy(rot_w, rot_x, rot_y, rot_z)
row_r, pitch_r, yaw_r = quaternion_to_rpy(rot_w_r, rot_x_r, rot_y_r, rot_z_r)
axs[1, 1].plot(row, color="black", label="r")
axs[1, 1].plot(pitch, color="blue", label="p")
axs[1, 1].plot(yaw, color="green", label="y")

axs[1, 1].plot(row_r, color="black", label="r_real", linestyle="dashed")
axs[1, 1].plot(pitch_r, color="blue", label="p_real", linestyle="dashed")
axs[1, 1].plot(yaw_r, color="green", label="y_real", linestyle="dashed")
axs[1, 1].legend()
axs[1, 1].set_title("Euler Angles")


# 3D 位置图
ax_3d = fig.add_subplot(2, 2, 3, projection='3d')
ax_3d.plot(pos_x, pos_y, pos_z, label="Position")
ax_3d.plot(pos_x_r, pos_y_r, pos_z_r, label="Reference Position")
ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')
ax_3d.legend()
ax_3d.set_xlim([-1.5, 1.5])
ax_3d.set_ylim([-1.5, 1.5])
ax_3d.set_zlim([0.0, 1.5])
# ax_3d.set_xlim([-1.5, 1.5])
# ax_3d.set_ylim([-1.5, 1.5])
# ax_3d.set_zlim([-5, 10])



# error
axs[2, 1].plot(error_pos_x, color="black", label="error_x")
axs[2, 1].plot(error_pos_y, color="blue", label="error_y")
axs[2, 1].plot(error_pos_z, color="green", label="error_z")
axs[2, 1].legend()
axs[2, 1].set_title("Position errors")

axs[3, 1].plot(error_vel_x, color="black", label="error_x")
axs[3, 1].plot(error_vel_y, color="blue", label="error_y")
axs[3, 1].plot(error_vel_z, color="green", label="error_z")
axs[3, 1].legend()
axs[3, 1].set_title("Velocity errors")

plt.show()

'''
fig_1, axs_1 = plt.subplots(2, 2, figsize=(15, 20))
fig_1.tight_layout(pad=4.0)

axs_1[0, 0].plot(error_pos_x_average, color="red", label="pos_error_x")
axs_1[0, 0].plot(error_pos_y_average, color="green", label="pos_error_y")
axs_1[0, 0].plot(error_pos_z_average, color="blue", label="pos_error_z")
axs_1[0, 0].legend()
axs_1[0, 0].set_title("Position Error")

axs_1[0, 1].plot(error_vel_x_average, color="red", label="vel_error_x")
axs_1[0, 1].plot(error_vel_y_average, color="green", label="vel_error_y")
axs_1[0, 1].plot(error_vel_z_average, color="blue", label="vel_error_z")
axs_1[0, 1].legend()
axs_1[0, 1].set_title("Velocity Error")


plt.show()
'''
