import numpy as np

from general_robotics_toolbox import *

def generate_trajectory_3d(waypoints, pub_rate, max_vel, max_acc, max_omg, max_alpha):
    """
    waypoints: list/array of (7,) [x,y,z,qw,qx,qy,qz], in your swarming frame
    Returns a dict with concatenated arrays sampled at pub_rate:
      t, p (Nx3), q (Nx4), v_lin (Nx3), a_lin (Nx3), omega (Nx3), alpha (Nx3), dt
    Assumption: stop at every waypoint (start/end at rest); within each segment,
    translation & rotation are synchronized to finish at the same time.
    """
    dt = 1.0 / float(pub_rate)
    wp = np.asarray(waypoints, float)
    assert wp.ndim == 2 and wp.shape[1] == 7, "Waypoints must be (N,7) [x,y,z,qw,qx,qy,qz]."

    # Output buffers (seed with first sample)
    T   = [0.0]
    P   = [wp[0, :3]]
    Q   = [wp[0, 3:7]]
    V   = [np.zeros(3)]
    A   = [np.zeros(3)]
    OMG = [np.zeros(3)]
    ALP = [np.zeros(3)]

    t_offset = 0.0
    for i in range(len(wp)-1):
        p0, q0 = wp[i, :3],  wp[i, 3:7]
        p1, q1 = wp[i+1, :3], wp[i+1, 3:7]

        seg = build_segment_3d(
            p0, q0, p1, q1,
            dt=dt,
            max_vel=max_vel, max_acc=max_acc,
            max_omg=max_omg, max_alpha=max_alpha
        )

        # Append (skip first sample to avoid duplication at joins)
        if len(seg['t']) > 1:
            t_rel = seg['t'][1:]
            T.extend((t_offset + t_rel).tolist())
            P.extend(seg['p'][1:].tolist())
            Q.extend(seg['q'][1:].tolist())
            V.extend(seg['v_lin'][1:].tolist())
            A.extend(seg['a_lin'][1:].tolist())
            OMG.extend(seg['omega'][1:].tolist())
            ALP.extend(seg['alpha'][1:].tolist())
            t_offset = T[-1]

    return dict(
        t=np.asarray(T),
        p=np.asarray(P),
        q=np.asarray(Q),
        v_lin=np.asarray(V),
        a_lin=np.asarray(A),
        omega=np.asarray(OMG),
        alpha=np.asarray(ALP),
        dt=dt
    )

# Generate waypoints for swarm trajectory
# Every waypoint is a numpy array of shape (7,) [x,y,z,qw,qx,qy,qz]
# Waypoints are represented in the swarming frame while "starting" the trajectory

## list of waypoints
waypoints = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])] # start at origin with no rotation
# move to 1m in x direction
waypoints.append(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
# move to 0.3m in y direction
waypoints.append(np.array([1.0, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0]))
# move to -0.3m in y direction
waypoints.append(np.array([1.0, -0.3, 0.0, 0.0, 0.0, 0.0, 1.0]))
# move to 0m in y direction
waypoints.append(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
# move to 0.4m in z direction
waypoints.append(np.array([1.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]))
# rotate 15 degrees around z axis
rot_quat = rot2q([0,0,1], np.deg2rad(15))
waypoints.append(np.array([1.0, 0.0, 0.4, rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]]))
# rotate -15 degrees around z axis
rot_quat = rot2q([0,0,1], np.deg2rad(-15))
waypoints.append(np.array([1.0, 0.0, 0.4, rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]]))
# rotate back to 0 degrees around z axis
waypoints.append(np.array([1.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]))
# move back 0.5m in x direction
waypoints.append(np.array([0.5, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]))
# move back to 0.0m in z direction
waypoints.append(np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
# move back to starting point
waypoints.append(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))

# convert the waypoints into trajectory
pub_rate = 200 # Hz
max_vel = 0.08 # m/s
max_acc = 0.2 # m/s^2
max_omg = np.deg2rad(8) # rad/s
max_alpha = np.deg2rad(10) # rad/s^2

# Build trajectory
traj = generate_trajectory_3d(waypoints, pub_rate, max_vel, max_acc, max_omg, max_alpha)
