import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

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
with open('output_control.txt') as file:
    data = file.readlines()

with open('output_diff_flat.txt') as file:
    data_diff_flat = file.readlines()

pos_x = np.array([float(data[i].split(", ")[0]) for i in range(len(data))])
pos_y = np.array([float(data[i].split(", ")[1]) for i in range(len(data))])
pos_z = np.array([float(data[i].split(", ")[2]) for i in range(len(data))])

pos_x_r = np.array([float(data[i].split(", ")[3]) for i in range(len(data))])
pos_y_r = np.array([float(data[i].split(", ")[4]) for i in range(len(data))])
pos_z_r = np.array([float(data[i].split(", ")[5]) for i in range(len(data))])

vel_x = np.array([float(data[i].split(", ")[6]) for i in range(len(data))])
vel_y = np.array([float(data[i].split(", ")[7]) for i in range(len(data))])
vel_z = np.array([float(data[i].split(", ")[8]) for i in range(len(data))])

vel_x_r = np.array([float(data[i].split(", ")[9]) for i in range(len(data))])
vel_y_r = np.array([float(data[i].split(", ")[10]) for i in range(len(data))])
vel_z_r = np.array([float(data[i].split(", ")[11]) for i in range(len(data))])

rot_w = np.array([float(data[i].split(", ")[12]) for i in range(len(data))])
rot_x = np.array([float(data[i].split(", ")[13]) for i in range(len(data))])
rot_y = np.array([float(data[i].split(", ")[14]) for i in range(len(data))])
rot_z = np.array([float(data[i].split(", ")[15]) for i in range(len(data))])

vel_ang_x = np.array([float(data[i].split(", ")[16]) for i in range(len(data))])
vel_ang_y = np.array([float(data[i].split(", ")[17]) for i in range(len(data))])
vel_ang_z = np.array([float(data[i].split(", ")[18]) for i in range(len(data))])

acc_ang_x = np.array([float(data[i].split(", ")[29]) for i in range(len(data))])
acc_ang_y = np.array([float(data[i].split(", ")[30]) for i in range(len(data))])
acc_ang_z = np.array([float(data[i].split(", ")[31]) for i in range(len(data))])


pos_x_diff_flat = np.array([float(data_diff_flat[i].split(", ")[0]) for i in range(len(data_diff_flat))])
pos_y_diff_flat = np.array([float(data_diff_flat[i].split(", ")[1]) for i in range(len(data_diff_flat))])
pos_z_diff_flat = np.array([float(data_diff_flat[i].split(", ")[2]) for i in range(len(data_diff_flat))])

vel_x_diff_flat = np.array([float(data_diff_flat[i].split(", ")[6]) for i in range(len(data_diff_flat))])
vel_y_diff_flat = np.array([float(data_diff_flat[i].split(", ")[7]) for i in range(len(data_diff_flat))])
vel_z_diff_flat = np.array([float(data_diff_flat[i].split(", ")[8]) for i in range(len(data_diff_flat))])

rot_w_diff_flat = np.array([float(data_diff_flat[i].split(", ")[12]) for i in range(len(data_diff_flat))])
rot_x_diff_flat = np.array([float(data_diff_flat[i].split(", ")[13]) for i in range(len(data_diff_flat))])
rot_y_diff_flat = np.array([float(data_diff_flat[i].split(", ")[14]) for i in range(len(data_diff_flat))])
rot_z_diff_flat = np.array([float(data_diff_flat[i].split(", ")[15]) for i in range(len(data_diff_flat))])

vel_ang_x_diff_flat = np.array([float(data_diff_flat[i].split(", ")[16]) for i in range(len(data_diff_flat))])
vel_ang_y_diff_flat = np.array([float(data_diff_flat[i].split(", ")[17]) for i in range(len(data_diff_flat))])
vel_ang_z_diff_flat = np.array([float(data_diff_flat[i].split(", ")[18]) for i in range(len(data_diff_flat))])

rot_w_r = np.array([float(data_diff_flat[i].split(", ")[19]) for i in range(len(data_diff_flat))])
rot_x_r = np.array([float(data_diff_flat[i].split(", ")[20]) for i in range(len(data_diff_flat))])
rot_y_r = np.array([float(data_diff_flat[i].split(", ")[21]) for i in range(len(data_diff_flat))])
rot_z_r = np.array([float(data_diff_flat[i].split(", ")[22]) for i in range(len(data_diff_flat))])

vel_ang_x_r = np.array([float(data_diff_flat[i].split(", ")[23]) for i in range(len(data_diff_flat))])
vel_ang_y_r = np.array([float(data_diff_flat[i].split(", ")[24]) for i in range(len(data_diff_flat))])
vel_ang_z_r = np.array([float(data_diff_flat[i].split(", ")[25]) for i in range(len(data_diff_flat))])

acc_ang_x_diff_flat = np.array([float(data_diff_flat[i].split(", ")[29]) for i in range(len(data_diff_flat))])
acc_ang_y_diff_flat = np.array([float(data_diff_flat[i].split(", ")[30]) for i in range(len(data_diff_flat))])
acc_ang_z_diff_flat = np.array([float(data_diff_flat[i].split(", ")[31]) for i in range(len(data_diff_flat))])

acc_ang_x_r = np.array([float(data_diff_flat[i].split(", ")[26]) for i in range(len(data_diff_flat))])
acc_ang_y_r = np.array([float(data_diff_flat[i].split(", ")[27]) for i in range(len(data_diff_flat))])
acc_ang_z_r = np.array([float(data_diff_flat[i].split(", ")[28]) for i in range(len(data_diff_flat))])


error_pos_x = pos_x_r - pos_x
error_pos_y = pos_y_r - pos_y
error_pos_z = pos_z_r - pos_z

error_vel_x = vel_x_r - vel_x
error_vel_y = vel_y_r - vel_y
error_vel_z = vel_z_r - vel_z


error_pos_x_diff_flat = pos_x_r - pos_x_diff_flat
error_pos_y_diff_flat = pos_y_r - pos_y_diff_flat
error_pos_z_diff_flat = pos_z_r - pos_z_diff_flat

error_vel_x_diff_flat = vel_x_r - vel_x_diff_flat
error_vel_y_diff_flat = vel_y_r - vel_y_diff_flat
error_vel_z_diff_flat = vel_z_r - vel_z_diff_flat

time = np.arange(0, len(data), 10)

row_control, pitch_control, yaw_control = quaternion_to_rpy(rot_w, rot_x, rot_y, rot_z)
row_diff_flat, pitch_diff_flat, yaw_diff_flat = quaternion_to_rpy(rot_w_diff_flat, rot_x_diff_flat, rot_y_diff_flat, rot_z_diff_flat)
row_ref, pitch_ref, yaw_ref = quaternion_to_rpy(rot_w_r, rot_x_r, rot_y_r, rot_z_r)


# 设置子图布局
fig, axs = plt.subplots(5, 3, figsize=(40, 30))
fig.tight_layout(pad=6.0)

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
axs[1, 0].plot(vel_x, color="black", label="vel_x")
axs[1, 0].plot(vel_y, color="blue", label="vel_y")
axs[1, 0].plot(vel_z, color="green", label="vel_z")
axs[1, 0].plot(vel_x_r, color="black", label="vel_x_real", linestyle="dashed")
axs[1, 0].plot(vel_y_r, color="blue", label="vel_y_real", linestyle="dashed")
axs[1, 0].plot(vel_z_r, color="green", label="vel_z_real", linestyle="dashed")
axs[1, 0].legend()
axs[1, 0].set_title("Velocity")


# 旋转图
axs[2, 0].plot(row_control, color="black", label="roll")
axs[2, 0].plot(pitch_control, color="blue", label="pitch")
axs[2, 0].plot(yaw_control, color="green", label="yaw")
axs[2, 0].plot(row_ref, color="black", label="roll", linestyle="dashed")
axs[2, 0].plot(pitch_ref, color="blue", label="pitch", linestyle="dashed")
axs[2, 0].plot(yaw_ref, color="green", label="yaw", linestyle="dashed")
axs[2, 0].legend()
axs[2, 0].set_title("Rotation")

# 角速度图
axs[3, 0].plot(vel_ang_x, color="black", label="ang_vel_x")
axs[3, 0].plot(vel_ang_y, color="blue", label="ang_vel_y")
axs[3, 0].plot(vel_ang_z, color="green", label="ang_vel_z")
axs[3, 0].plot(vel_ang_x_r, color="black", label="ang_vel_x", linestyle="dashed")
axs[3, 0].plot(vel_ang_y_r, color="blue", label="ang_vel_y", linestyle="dashed")
axs[3, 0].plot(vel_ang_z_r, color="green", label="ang_vel_z", linestyle="dashed")
axs[3, 0].legend()
axs[3, 0].set_title("Angular Velocity")

# 角加速度图
axs[4, 0].plot(acc_ang_x, color="black", label="ang_acc_x")
axs[4, 0].plot(acc_ang_y, color="blue", label="ang_acc_y")
axs[4, 0].plot(acc_ang_z, color="green", label="ang_acc_z")
axs[4, 0].plot(acc_ang_x_r, color="black", label="ang_acc_x", linestyle="dashed")
axs[4, 0].plot(acc_ang_y_r, color="blue", label="ang_acc_y", linestyle="dashed")
axs[4, 0].plot(acc_ang_z_r, color="green", label="ang_acc_z", linestyle="dashed")
axs[4, 0].legend()
axs[4, 0].set_title("Angular Acceleration")


# 位置图
axs[0, 1].plot(pos_x_diff_flat, color="black", label="pos_x")
axs[0, 1].plot(pos_y_diff_flat, color="blue", label="pos_y")
axs[0, 1].plot(pos_z_diff_flat, color="green", label="pos_z")
axs[0, 1].plot(pos_x_r, color="black", label="pos_x_real", linestyle="dashed")
axs[0, 1].plot(pos_y_r, color="blue", label="pos_y_real", linestyle="dashed")
axs[0, 1].plot(pos_z_r, color="green", label="pos_z_real", linestyle="dashed")
axs[0, 1].legend()
axs[0, 1].set_title("DF Position")

# 速度图
axs[1, 1].plot(vel_x_diff_flat, color="black", label="vel_x")
axs[1, 1].plot(vel_y_diff_flat, color="blue", label="vel_y")
axs[1, 1].plot(vel_z_diff_flat, color="green", label="vel_z")
axs[1, 1].plot(vel_x_r, color="black", label="vel_x_real", linestyle="dashed")
axs[1, 1].plot(vel_y_r, color="blue", label="vel_y_real", linestyle="dashed")
axs[1, 1].plot(vel_z_r, color="green", label="vel_z_real", linestyle="dashed")
axs[1, 1].legend()
axs[1, 1].set_title("DF Velocity")

# 旋转图
axs[2, 1].plot(row_diff_flat, color="black", label="roll")
axs[2, 1].plot(pitch_diff_flat, color="blue", label="pitch")
axs[2, 1].plot(yaw_diff_flat, color="green", label="yaw")
axs[2, 1].plot(row_ref, color="black", label="roll", linestyle="dashed")
axs[2, 1].plot(pitch_ref, color="blue", label="pitch", linestyle="dashed")
axs[2, 1].plot(yaw_ref, color="green", label="yaw", linestyle="dashed")
axs[2, 1].legend()
axs[2, 1].set_title("DF Rotation")

# 角速度图
axs[3, 1].plot(vel_ang_x_diff_flat, color="black", label="ang_vel_x")
axs[3, 1].plot(vel_ang_y_diff_flat, color="blue", label="ang_vel_y")
axs[3, 1].plot(vel_ang_z_diff_flat, color="green",label="ang_vel_z")
axs[3, 1].plot(vel_ang_x_r, color="black", label="ang_vel_x", linestyle="dashed")
axs[3, 1].plot(vel_ang_y_r, color="blue", label="ang_vel_y", linestyle="dashed")
axs[3, 1].plot(vel_ang_z_r, color="green", label="ang_vel_z", linestyle="dashed")
axs[3, 1].legend()
axs[3, 1].set_title("DF Angular Velocity")

# 角加速度图
axs[4, 1].plot(acc_ang_x_diff_flat, color="black", label="ang_acc_x")
axs[4, 1].plot(acc_ang_y_diff_flat, color="blue", label="ang_acc_y")
axs[4, 1].plot(acc_ang_z_diff_flat, color="green", label="ang_acc_z")
axs[4, 1].plot(acc_ang_x_r, color="black", label="ang_acc_x", linestyle="dashed")
axs[4, 1].plot(acc_ang_y_r, color="blue", label="ang_acc_y", linestyle="dashed")
axs[4, 1].plot(acc_ang_z_r, color="green", label="ang_acc_z", linestyle="dashed")
axs[4, 1].legend()
axs[4, 1].set_title("Angular Acceleration")

# error
axs[0, 2].plot(error_pos_x, color="black", label="error_x")
axs[0, 2].plot(error_pos_y, color="blue", label="error_y")
axs[0, 2].plot(error_pos_z, color="green", label="error_z")
axs[0, 2].legend()
axs[0, 2].set_title("Control Position errors")

axs[1, 2].plot(error_vel_x, color="black", label="error_x")
axs[1, 2].plot(error_vel_y, color="blue", label="error_y")
axs[1, 2].plot(error_vel_z, color="green", label="error_z")
axs[1, 2].legend()
axs[1, 2].set_title("Control Velocity errors")

axs[2, 2].plot(error_pos_x_diff_flat, color="black", label="error_x")
axs[2, 2].plot(error_pos_y_diff_flat, color="blue", label="error_y")
axs[2, 2].plot(error_pos_z_diff_flat, color="green", label="error_z")
axs[2, 2].legend()
axs[2, 2].set_title("DF Position errors")

axs[3, 2].plot(error_vel_x_diff_flat, color="black", label="error_x")
axs[3, 2].plot(error_vel_y_diff_flat, color="blue", label="error_y")
axs[3, 2].plot(error_vel_z_diff_flat, color="green", label="error_z")
axs[3, 2].legend()
axs[3, 2].set_title("DF Velocity errors")

# plt.show()

# 创建新的 3D 图像
fig_3d = plt.figure(figsize=(8, 6))  # 可以调整图的大小
ax_3d = fig_3d.add_subplot(111, projection='3d')  # 直接创建 3D 轴

# 绘制轨迹
ax_3d.plot(pos_x, pos_y, pos_z, color="blue", label="Control Trajectory")
ax_3d.plot(pos_x_diff_flat, pos_y_diff_flat, pos_z_diff_flat, color="black", label="DF Trajectory")
ax_3d.plot(pos_x_r, pos_y_r, pos_z_r, color="green",label="Reference Trajectory")

# 坐标轴标签
ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')

# 设置图例
ax_3d.legend()

# 设置坐标轴范围
'''
ax_3d.set_xlim([-1.5, 1.5])
ax_3d.set_ylim([-1.5, 1.5])
ax_3d.set_zlim([0.0, 1.5])
'''

# 显示独立的 3D 图
plt.show()