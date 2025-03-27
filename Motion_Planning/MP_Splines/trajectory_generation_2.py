import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import csv

# Evaluate a cubic spline with coefficients a[0] ... a[3] at t in [0,1]
def eval(a, t):
    return a[:,0] + a[:,1] * t + a[:,2] * t**2 + a[:,3] * t**3 + a[:,4] * t**4 + a[:,5] * t**5 + a[:,6] * t**6 + a[:,7] * t**7 + a[:,8] * t**8

# Evaluate the first derivative of the 8th-order polynomial splines with coefficients a[0] ... a[7] at t in [0,1]
def eval_p(a, t):
    return a[:,1] + 2 * a[:,2] * t + 3 * a[:,3] * t**2 + 4 * a[:,4] * t**3 + 5 * a[:,5] * t**4 + 6 * a[:,6] * t**5 + 7 * a[:,7] * t**6 + 8 * a[:,8] * t**7
# Evaluate the second derivative of the 8th-order polynomial splines with coefficients a[0] ... a[7] at t in [0,1] acceleration
def eval_pp(a, t):
    return 2 * a[:,2] + 6 * a[:,3] * t + 12 * a[:,4] * t**2 + 20 * a[:,5] * t**3 + 30 * a[:,6] * t**4 + 42 * a[:,7] * t**5 + 56 * a[:,8] * t**6

# Evaluate the third derivative of the 8th-order polynomial splines with coefficients a[0] ... a[7] at t in [0,1] jerk
def eval_ppp(a, t):
    return 6 * a[:,3] + 24 * a[:,4] * t + 60 * a[:,5] * t**2 + 120 * a[:,6] * t**3 + 210 * a[:,7] * t**4 + 336 * a[:,8] * t**5
# Evaluate the fourth derivative of the 8th-order polynomial splines with coefficients a[0] ... a[7] at t in [0,1] snap
def eval_pppp(a, t):
    return 24 * a[:,4] + 120 * a[:,5] * t + 360 * a[:,6] * t**2 + 840 * a[:,7] * t**3 + 1680 * a[:,8] * t**4

# Compute acceleration cost of cubic spline with coefficients a[0] ... a[3]

def cost_acc(a):
    return (2 * a[:,2] + 3 * a[:,3] + 4 * a[:,4] + 5 * a[:,5] + 6 * a[:,6] + 7 * a[:,7] + a[:,8]) **2


def main():
    points = np.array([
        [0, 0,0],
        [0.0, 0.5,0],
        [0., 1., 0.],
    ])

    # Construct the problem.
    a = cp.Variable((3,9))
    b = cp.Variable((3,9))
    c = cp.Variable((3,9))
    p2 = cp.Variable((3,))
    objective = cp.Minimize(cp.sum(cost_acc(a) + cost_acc(c) + cost_acc(b)))
    constraints = [
        eval(a, 0) == points[0],
        eval(a, 1) == points[1],
        eval(c, 0) == points[1],

        eval(b, 0) == eval(a,1),
        eval(b,1) == p2,
        eval(c, 0) == p2,
        eval(c, 1) == points[2],

        #smoothness
        eval_p(a, 1) == eval_p(b, 0),
        eval_pp(a, 1) == eval_pp(b, 0),
        eval_ppp(a, 1) == eval_ppp(b, 0),
        eval_pppp(a, 1) == eval_pppp(b, 0),

        eval_p(b, 1) == eval_p(c, 0),
        eval_pp(b, 1) == eval_pp(c, 0),
        eval_ppp(b, 1) == eval_ppp(c, 0),
        eval_pppp(b, 1) == eval_pppp(c, 0),

        #endpoint smoothness
        eval_p(a, 0) == 0,
        eval_pp(a, 0) == 0,
        eval_ppp(a, 0) == 0,
        eval_pppp(a, 0) == 0,

        eval_p(c, 1) == 0, 
        eval_pp(c, 1) == 0,
        eval_ppp(c, 1) == 0,
        eval_pppp(c, 1) == 0,
  
        eval_pp(a, 1) == [8.66025404, 0., -4.81]

        
    ]

    #TODO: ADD aggressive maneuvering constraints using 
    #pp  = g + r *R_d * e3, where r ∈ R+

    
    

    #unbounded tips: add more constraints, like ppppp
    #more segments
    #more constraints in segment waypoints
    #endpoints maybe not so many constraints
    
    prob = cp.Problem(objective, constraints)

    # The optimal objective value is returned by `prob.solve()`.
    result = prob.solve(solver=cp.OSQP, verbose=True)
    # The optimal value for x is stored in `x.value`.

    ts = np.linspace(0, 1, 100)
    p_a = np.array([eval(a.value, t) for t in ts])
    p_b = np.array([eval(b.value, t) for t in ts])
    p_c = np.array([eval(c.value, t) for t in ts])


    # 3D Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the two spline segments in 3D
    ax.plot(p_a[:, 0], p_a[:, 1], p_a[:, 2], label="Segment A")
    ax.plot(p_b[:, 0], p_b[:, 1], p_b[:, 2], label="Segment B")
    ax.plot(p_c[:, 0], p_c[:, 1], p_c[:, 2], label="Segment C")

    # Plot the original points in 3D
    #extract the via points from the solution
    waypoints = np.array([p_a[0], p_a[-1], p_b[0], p_b[-1], p_c[0], p_c[-1]])
    #ax.scatter(points[:,0], points[:,1], points[:,2], c='red', marker='o', label="Waypoints")
    ax.scatter(waypoints[:,0], waypoints[:,1], waypoints[:,2], c='red', marker='o', label="Waypoints")

    # Label axes and show legend
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    #save the plot
    plt.savefig("spline_3d.png")

    #save the trajectory to a csv file with the header
    #t,posx,posy,posz,velx,vely,velz,accx,accy,accz,jerkx,jerky,jerkz,snapx,snapy,snapz
    #0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,19.930632000000003,-32.546568,0.0
    #one timestep should be 0.01 seconds for each trajectory piece?
    with open("trajectory.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t","posx","posy","posz","velx","vely","velz","accx","accy","accz",
                        "jerkx","jerky","jerkz","snapx","snapy","snapz"])

        # Sample each segment with 100 points, one point being every 0.01 seconds
        ts = np.linspace(0, 1, 100)
        t_global = 0.0
        for segment_coeffs in [a.value, b.value, c.value]:
            for t_step in ts:
                pos = eval(segment_coeffs, t_step)
                vel = eval_p(segment_coeffs, t_step)
                acc = eval_pp(segment_coeffs, t_step)
                jerk = eval_ppp(segment_coeffs, t_step)
                snap = eval_pppp(segment_coeffs, t_step)
                row = [
                    t_global,
                    pos[0], pos[1], pos[2],
                    vel[0], vel[1], vel[2],
                    acc[0], acc[1], acc[2],
                    jerk[0], jerk[1], jerk[2],
                    snap[0], snap[1], snap[2]
                ]
                writer.writerow(row)
                t_global += (1.0 / 99)  
    
    plt.show()



if __name__ == "__main__":
  main()