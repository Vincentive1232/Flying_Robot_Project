import cvxpy as cp
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd


def eval_spline8(a, t):
    return (a[:, 0] +
            a[:, 1] * t +
            a[:, 2] * math.pow(t, 2) +
            a[:, 3] * math.pow(t, 3) +
            a[:, 4] * math.pow(t, 4) +
            a[:, 5] * math.pow(t, 5) +
            a[:, 6] * math.pow(t, 6) +
            a[:, 7] * math.pow(t, 7) +
            a[:, 8] * math.pow(t, 8))

def eval_spline8_z(a, t):
    return (a[2, 0] +
            a[2, 1] * t +
            a[2, 2] * math.pow(t, 2) +
            a[2, 3] * math.pow(t, 3) +
            a[2, 4] * math.pow(t, 4) +
            a[2, 5] * math.pow(t, 5) +
            a[2, 6] * math.pow(t, 6) +
            a[2, 7] * math.pow(t, 7) +
            a[2, 8] * math.pow(t, 8))

def eval_spline8_d(a, t):
    return (a[:, 1] +
            2 * a[:, 2] * t +
            3 * a[:, 3] * math.pow(t, 2) +
            4 * a[:, 4] * math.pow(t, 3) +
            5 * a[:, 5] * math.pow(t, 4) +
            6 * a[:, 6] * math.pow(t, 5) +
            7 * a[:, 7] * math.pow(t, 6) +
            8 * a[:, 8] * math.pow(t, 7))


def eval_spline8_dd(a, t):
    return (2 * a[:, 2] +
            3 * 2 * a[:, 3] * t +
            4 * 3 * a[:, 4] * math.pow(t, 2) +
            5 * 4 * a[:, 5] * math.pow(t, 3) +
            6 * 5 * a[:, 6] * math.pow(t, 4) +
            7 * 6 * a[:, 7] * math.pow(t, 5) +
            8 * 7 * a[:, 8] * math.pow(t, 6))


def eval_spline8_ddd(a, t):
    return (3 * 2 * 1 * a[:, 3] +
            4 * 3 * 2 * a[:, 4] * t +
            5 * 4 * 3 * a[:, 5] * math.pow(t, 2) +
            6 * 5 * 4 * a[:, 6] * math.pow(t, 3) +
            7 * 6 * 5 * a[:, 7] * math.pow(t, 4) +
            8 * 7 * 6 * a[:, 8] * math.pow(t, 5))


def eval_spline8_dddd(a, t):
    return (4 * 3 * 2 * 1 * a[:, 4] +
            5 * 4 * 3 * 2 * a[:, 5] * t +
            6 * 5 * 4 * 3 * a[:, 6] * math.pow(t, 2) +
            7 * 6 * 5 * 4 * a[:, 7] * math.pow(t, 3) +
            8 * 7 * 6 * 5 * a[:, 8] * math.pow(t, 4))


def cost_acc_spline8(a):
    return (2 * a[:, 2] +
            3 * a[:, 3] +
            4 * a[:, 4] +
            5 * a[:, 5] +
            6 * a[:, 6] +
            7 * a[:, 7] +
            8 * a[:, 8])


def save_2_csv(t, posx, posy, posz, velx, vely, velz, accx, accy, accz, jerkx, jerky, jerkz, snapx, snapy, snapz):
    data = {
        "t": t,
        "posx": posx, "posy": posy, "posz": posz,
        "velx": velx, "vely": vely, "velz": velz,
        "accx": accx, "accy": accy, "accz": accz,
        "jerkx": jerkx, "jerky": jerky, "jerkz": jerkz,
        "snapx": snapx, "snapy": snapy, "snapz": snapz,
    }

    # 创建 DataFrame
    df = pd.DataFrame(data)

    # 保存为 CSV 文件，不带索引
    df.to_csv("aggressive_traj_0001_3.csv", index=False)

    print("aggressive_traj.csv has been successfully saved")


def PathPoint_Generating():
    '''
    points = np.array([
        [0, 0, 0],
        [0, 0.5, 0],
        [0, 1, 0]
    ])
    acceleration_constraints = np.array([
        [8.66025404, 0., -4.81]
    ])
    '''
    points = np.array([
        [0, 0, 0],
        [1, 1, 0.5],
        [2, 0, 1]
    ])
    acceleration_constraints = np.array([
        [2, 2, 1.5]
    ])

    a = cp.Variable((3, 9))
    b = cp.Variable((3, 9))

    objective = cp.Minimize(cp.sum(cost_acc_spline8(a) +
                                   cost_acc_spline8(b)))
    constraints = [
        eval_spline8(a, 0) == points[0],             # First Spline start from the start position
        eval_spline8(a, 1) == points[1],             # First Spline end in the middle point
        eval_spline8(b, 0) == points[1],             # Second Spline start from the middle point
        eval_spline8(b, 1) == points[2],             # End in target position
        eval_spline8_d(a, 1) == eval_spline8_d(b, 0),
        eval_spline8_dd(a, 1) == eval_spline8_dd(b, 0),
        eval_spline8_dd(a, 1) == acceleration_constraints[0],
        eval_spline8_dd(b, 0) == acceleration_constraints[0],
        eval_spline8_ddd(a, 1) == eval_spline8_ddd(b, 0),
        eval_spline8_dddd(a, 1) == eval_spline8_dddd(b, 0),
        eval_spline8_d(a, 0) == 0,                   # Start velocity is zero
        eval_spline8_dd(a, 0) == 0,
        eval_spline8_ddd(a, 0) == 0,
        eval_spline8_dddd(a, 0) == 0,
        eval_spline8_d(b, 1) == 0,                   # End velocity is zero
        eval_spline8_dd(b, 1) == 0,
        eval_spline8_ddd(b, 1) == 0,
        eval_spline8_dddd(b, 1) == 0
    ]

    prob = cp.Problem(objective, constraints)

    result = prob.solve(verbose=True)

    ts = np.linspace(0, 1, 10000)
    p_a = np.array([eval_spline8(a.value, t) for t in ts])
    p_b = np.array([eval_spline8(b.value, t) for t in ts])

    v_a = np.array([eval_spline8_d(a.value, t) for t in ts])
    v_b = np.array([eval_spline8_d(b.value, t) for t in ts])

    a_a = np.array([eval_spline8_dd(a.value, t) for t in ts])
    a_b = np.array([eval_spline8_dd(b.value, t) for t in ts])

    j_a = np.array([eval_spline8_ddd(a.value, t) for t in ts])
    j_b = np.array([eval_spline8_ddd(b.value, t) for t in ts])

    s_a = np.array([eval_spline8_dddd(a.value, t) for t in ts])
    s_b = np.array([eval_spline8_dddd(b.value, t) for t in ts])

    pos_x = np.concatenate([p_a[:, 0], p_b[:, 0]])
    pos_y = np.concatenate([p_a[:, 1], p_b[:, 1]])
    pos_z = np.concatenate([p_a[:, 2], p_b[:, 2]])

    vel_x = np.concatenate([v_a[:, 0], v_b[:, 0]])
    vel_y = np.concatenate([v_a[:, 1], v_b[:, 1]])
    vel_z = np.concatenate([v_a[:, 2], v_b[:, 2]])

    acc_x = np.concatenate([a_a[:, 0], a_b[:, 0]])
    acc_y = np.concatenate([a_a[:, 1], a_b[:, 1]])
    acc_z = np.concatenate([a_a[:, 2], a_b[:, 2]])

    jerk_x = np.concatenate([j_a[:, 0], j_b[:, 0]])
    jerk_y = np.concatenate([j_a[:, 1], j_b[:, 1]])
    jerk_z = np.concatenate([j_a[:, 2], j_b[:, 2]])

    snap_x = np.concatenate([s_a[:, 0], s_b[:, 0]])
    snap_y = np.concatenate([s_a[:, 1], s_b[:, 1]])
    snap_z = np.concatenate([s_a[:, 2], s_b[:, 2]])

    t_horizon = np.linspace(0, 2, 20000)
    save_2_csv(t_horizon, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, jerk_x, jerk_y, jerk_z, snap_x, snap_y, snap_z)

    # 3D Plot
    fig = plt.figure(figsize=(6, 6))
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.plot(p_a[:, 0], p_a[:, 1], p_a[:, 2], label="Seg 1")
    ax_3d.plot(p_b[:, 0], p_b[:, 1], p_b[:, 2], label="Seg 2")
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.legend()
    '''
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([0.0, 1.5])
    '''

    ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], s=20)
    plt.show()


def PathPoint_Generating_3():
    '''
    points = np.array([
        [0, 0, 0],
        [0, 0.5, 0],
        [0, 1, 0]
    ])
    acceleration_constraints = np.array([
        [8.66025404, 0., -4.81]
    ])
    '''
    points = np.array([
        [0, 0, 0],
        [0.7, 0.3, 0.2],
        [1.4, 0.6, 0.5],
        [2, 0, 1]
    ])
    acceleration_constraints = np.array([
        [0.8, 0.8, 0.4],
        [1.0, 1.0, 0.5]
    ])

    a = cp.Variable((3, 9))
    b = cp.Variable((3, 9))
    c = cp.Variable((3, 9))

    objective = cp.Minimize(cp.sum(cost_acc_spline8(a) +
                                   cost_acc_spline8(b) +
                                   cost_acc_spline8(c)))
    constraints = [
        eval_spline8(a, 0) == points[0],             # First Spline start from the start position
        eval_spline8(a, 1) == points[1],             # First Spline end in the middle point
        eval_spline8(b, 0) == points[1],             # Second Spline start from the middle point
        eval_spline8(b, 1) == points[2],             # End in target position
        eval_spline8(c, 0) == points[2],             # End in target position
        eval_spline8(c, 1) == points[3],

        eval_spline8_d(a, 1) == eval_spline8_d(b, 0),
        eval_spline8_dd(a, 1) == eval_spline8_dd(b, 0),
        eval_spline8_dd(a, 1) == acceleration_constraints[0],
        eval_spline8_dd(b, 0) == acceleration_constraints[0],
        eval_spline8_ddd(a, 1) == eval_spline8_ddd(b, 0),
        eval_spline8_dddd(a, 1) == eval_spline8_dddd(b, 0),

        eval_spline8_d(b, 1) == eval_spline8_d(c, 0),
        eval_spline8_dd(b, 1) == eval_spline8_dd(c, 0),
        eval_spline8_dd(b, 1) == acceleration_constraints[1],
        eval_spline8_dd(c, 0) == acceleration_constraints[1],
        eval_spline8_ddd(b, 1) == eval_spline8_ddd(c, 0),
        eval_spline8_dddd(b, 1) == eval_spline8_dddd(c, 0),

        eval_spline8_d(a, 0) == 0,                   # Start velocity is zero
        eval_spline8_dd(a, 0) == 0,
        eval_spline8_ddd(a, 0) == 0,
        eval_spline8_dddd(a, 0) == 0,
        eval_spline8_d(c, 1) == 0,                   # End velocity is zero
        eval_spline8_dd(c, 1) == 0,
        eval_spline8_ddd(c, 1) == 0,
        eval_spline8_dddd(c, 1) == 0
    ]

    prob = cp.Problem(objective, constraints)

    result = prob.solve(verbose=True)

    ts = np.linspace(0, 1, 10000)
    p_a = np.array([eval_spline8(a.value, t) for t in ts])
    p_b = np.array([eval_spline8(b.value, t) for t in ts])
    p_c = np.array([eval_spline8(c.value, t) for t in ts])

    v_a = np.array([eval_spline8_d(a.value, t) for t in ts])
    v_b = np.array([eval_spline8_d(b.value, t) for t in ts])
    v_c = np.array([eval_spline8_d(c.value, t) for t in ts])

    a_a = np.array([eval_spline8_dd(a.value, t) for t in ts])
    a_b = np.array([eval_spline8_dd(b.value, t) for t in ts])
    a_c = np.array([eval_spline8_dd(c.value, t) for t in ts])

    j_a = np.array([eval_spline8_ddd(a.value, t) for t in ts])
    j_b = np.array([eval_spline8_ddd(b.value, t) for t in ts])
    j_c = np.array([eval_spline8_ddd(c.value, t) for t in ts])

    s_a = np.array([eval_spline8_dddd(a.value, t) for t in ts])
    s_b = np.array([eval_spline8_dddd(b.value, t) for t in ts])
    s_c = np.array([eval_spline8_dddd(c.value, t) for t in ts])

    pos_x = np.concatenate([p_a[:, 0], p_b[:, 0], p_c[:, 0]])
    pos_y = np.concatenate([p_a[:, 1], p_b[:, 1], p_c[:, 1]])
    pos_z = np.concatenate([p_a[:, 2], p_b[:, 2], p_c[:, 2]])

    vel_x = np.concatenate([v_a[:, 0], v_b[:, 0], v_c[:, 0]])
    vel_y = np.concatenate([v_a[:, 1], v_b[:, 1], v_c[:, 1]])
    vel_z = np.concatenate([v_a[:, 2], v_b[:, 2], v_c[:, 2]])

    acc_x = np.concatenate([a_a[:, 0], a_b[:, 0], a_c[:, 0]])
    acc_y = np.concatenate([a_a[:, 1], a_b[:, 1], a_c[:, 1]])
    acc_z = np.concatenate([a_a[:, 2], a_b[:, 2], a_c[:, 2]])

    jerk_x = np.concatenate([j_a[:, 0], j_b[:, 0], j_c[:, 0]])
    jerk_y = np.concatenate([j_a[:, 1], j_b[:, 1], j_c[:, 1]])
    jerk_z = np.concatenate([j_a[:, 2], j_b[:, 2], j_c[:, 2]])

    snap_x = np.concatenate([s_a[:, 0], s_b[:, 0], s_c[:, 0]])
    snap_y = np.concatenate([s_a[:, 1], s_b[:, 1], s_c[:, 1]])
    snap_z = np.concatenate([s_a[:, 2], s_b[:, 2], s_c[:, 2]])

    t_horizon = np.linspace(0, 3, 30000)
    save_2_csv(t_horizon, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, jerk_x, jerk_y, jerk_z, snap_x, snap_y, snap_z)

    # 3D Plot
    fig = plt.figure(figsize=(6, 6))
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.plot(p_a[:, 0], p_a[:, 1], p_a[:, 2], label="Seg 1")
    ax_3d.plot(p_b[:, 0], p_b[:, 1], p_b[:, 2], label="Seg 2")
    ax_3d.plot(p_c[:, 0], p_c[:, 1], p_c[:, 2], label="Seg 3")
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.legend()
    '''
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([0.0, 1.5])
    '''

    ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], s=20)
    plt.show()


if __name__ == "__main__":
    PathPoint_Generating_3()
   # PathPoint_Generating()
