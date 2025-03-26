import numpy as np
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


# 读取数据
with open('output.txt') as file:
    data = file.readlines()

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

rot_w_r = np.array([float(data[i].split(", ")[16]) for i in range(len(data))])
rot_x_r = np.array([float(data[i].split(", ")[17]) for i in range(len(data))])
rot_y_r = np.array([float(data[i].split(", ")[18]) for i in range(len(data))])
rot_z_r = np.array([float(data[i].split(", ")[19]) for i in range(len(data))])

vel_ang_x = np.array([float(data[i].split(", ")[20]) for i in range(len(data))])
vel_ang_y = np.array([float(data[i].split(", ")[21]) for i in range(len(data))])
vel_ang_z = np.array([float(data[i].split(", ")[22]) for i in range(len(data))])

vel_ang_x_r = np.array([float(data[i].split(", ")[23]) for i in range(len(data))])
vel_ang_y_r = np.array([float(data[i].split(", ")[24]) for i in range(len(data))])
vel_ang_z_r = np.array([float(data[i].split(", ")[25]) for i in range(len(data))])


error_pos_x = np.abs(pos_x_r - pos_x)
error_pos_y = np.abs(pos_y_r - pos_y)
error_pos_z = np.abs(pos_z_r - pos_z)

error_vel_x = np.abs(vel_x_r - vel_x)
error_vel_y = np.abs(vel_y_r - vel_y)
error_vel_z = np.abs(vel_z_r - vel_z)

q_plus_p_qw = rot_w_r + rot_w
q_plus_p_qx = rot_x_r + rot_x
q_plus_p_qy = rot_y_r + rot_y
q_plus_p_qz = rot_z_r + rot_z
q_sub_p_qw = rot_w_r - rot_w
q_sub_p_qx = rot_x_r - rot_x
q_sub_p_qy = rot_y_r - rot_y
q_sub_p_qz = rot_z_r - rot_z

q_sub_p_error = np.sqrt((q_sub_p_qw**2)+(q_sub_p_qx**2)+(q_sub_p_qy**2)+(q_sub_p_qz**2))
q_plus_p_error = np.sqrt((q_plus_p_qw**2)+(q_plus_p_qx**2)+(q_plus_p_qy**2)+(q_plus_p_qz**2))
q_error = np.minimum(q_plus_p_error, q_sub_p_error)

error_ang_vel_x = np.abs(vel_ang_x_r - vel_ang_x)
error_ang_vel_y = np.abs(vel_ang_y_r - vel_ang_y)
error_ang_vel_z = np.abs(vel_ang_z_r - vel_ang_z)

time = np.arange(0, len(data), 10)

error_pos_x_average = average_every_ten(error_pos_x)
error_pos_y_average = average_every_ten(error_pos_y)
error_pos_z_average = average_every_ten(error_pos_z)

error_vel_x_average = average_every_ten(error_vel_x)
error_vel_y_average = average_every_ten(error_vel_y)
error_vel_z_average = average_every_ten(error_vel_z)

error_ang_vel_x_average = average_every_ten(error_ang_vel_x)
error_ang_vel_y_average = average_every_ten(error_ang_vel_y)
error_ang_vel_z_average = average_every_ten(error_ang_vel_z)


        

        

# 设置子图布局
fig, axs = plt.subplots(4, 2, figsize=(15, 20))
fig.tight_layout(pad=4.0)

# 位置图
axs[0, 0].plot(pos_x, color="red", label="pos_x")
axs[0, 0].plot(pos_y, color="green", label="pos_y")
axs[0, 0].plot(pos_z, color="blue", label="pos_z")
axs[0, 0].plot(pos_x_r, color="red", label="pos_x_real", linestyle="dashed")
axs[0, 0].plot(pos_y_r, color="green", label="pos_y_real", linestyle="dashed")
axs[0, 0].plot(pos_z_r, color="blue", label="pos_z_real", linestyle="dashed")
axs[0, 0].legend()
axs[0, 0].set_title("Position")

# 速度图
axs[0, 1].plot(vel_x, color="red", label="vel_x")
axs[0, 1].plot(vel_y, color="green", label="vel_y")
axs[0, 1].plot(vel_z, color="blue", label="vel_z")
axs[0, 1].plot(vel_x_r, color="red", label="vel_x_real", linestyle="dashed")
axs[0, 1].plot(vel_y_r, color="green", label="vel_y_real", linestyle="dashed")
axs[0, 1].plot(vel_z_r, color="blue", label="vel_z_real", linestyle="dashed")
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

# 角速度图
axs[1, 1].plot(vel_ang_x, color="red", label="ang_vel_x")
axs[1, 1].plot(vel_ang_y, color="green", label="ang_vel_y")
axs[1, 1].plot(vel_ang_z, color="blue",label="ang_vel_z")
axs[1, 1].plot(vel_ang_x_r, color="red", label="ang_vel_x_real", linestyle="dashed")
axs[1, 1].plot(vel_ang_y_r, color="green", label="ang_vel_x_real", linestyle="dashed")
axs[1, 1].plot(vel_ang_z_r, color="blue", label="ang_vel_x_real", linestyle="dashed")
axs[1, 1].legend()
axs[1, 1].set_title("Angular Velocity")

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

# 单独的 Z 位置与旋转图
axs[2, 1].plot(pos_z, color="black", label="Pos Z")
axs[2, 1].plot(rot_x, color="red", label="Rot X")
axs[2, 1].plot(rot_y, color="green", label="Rot Y")
axs[2, 1].plot(rot_z, color="blue", label="Rot Z")
axs[2, 1].legend()
axs[2, 1].set_title("Position Z and Rotation")

plt.show()

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

axs_1[1, 0].plot(q_error, color="red", label="quaternion_error")
axs_1[1, 0].legend()
axs_1[1, 0].set_title("Angular Error")

axs_1[1, 1].plot(error_ang_vel_x_average, color="red", label="ang_vel_error_x")
axs_1[1, 1].plot(error_ang_vel_y_average, color="green", label="ang_vel_error_y")
axs_1[1, 1].plot(error_ang_vel_z_average, color="blue", label="ang_vel_error_z")
axs_1[1, 1].legend()
axs_1[1, 1].set_title("Angular Velocity Error")


plt.show()
