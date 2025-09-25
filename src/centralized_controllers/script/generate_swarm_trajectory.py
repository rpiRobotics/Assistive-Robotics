import numpy as np

from general_robotics_toolbox import *

def _angle_wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def trapezoid_min_time(L, v_max, a_max):
    """
    Minimal time to cover distance L with accel limit a_max and speed cap v_max.
    v_peak* = min(v_max, sqrt(L*a_max))
    t_min = L/v_peak + v_peak/a_max
    """
    if L <= 0:
        return 0.0
    v_peak = min(v_max, np.sqrt(L * a_max))
    return L / v_peak + v_peak / a_max

def solve_vpeak_for_time(L, a_max, t_total):
    """
    Given distance L, accel a_max and a desired total time t_total >= 2*sqrt(L/a),
    find the peak velocity (<= unconstrained) so that the motion is trapezoid/triangle
    with that total time. Formula from t = L/v + v/a  -> v^2 - a t v + a L = 0.
    """
    disc = (a_max * t_total)**2 - 4.0 * a_max * L
    if disc < -1e-9:
        # No solution; caller should avoid this (t_total too small)
        raise ValueError("Requested total time is infeasible.")
    disc = max(0.0, disc)
    v = 0.5 * (a_max * t_total - np.sqrt(disc))
    return max(1e-12, v)

def trapezoid_from_vpeak(L, a_max, v_peak, dt):
    """
    Build a 1D profile (t, s, v, a) from distance L, accel a_max and chosen v_peak.
    Works for both triangular (if v_peak == sqrt(L*a)) and trapezoidal.
    """
    if L <= 0.0:
        z = np.array([0.0])
        return z, z, z, z

    # Time to accelerate to v_peak
    t_acc = v_peak / a_max
    d_acc = 0.5 * a_max * t_acc**2

    if 2*d_acc >= L - 1e-12:
        # Triangular (never reach plateau). Adjust v_peak to triangular value for exactness
        t_acc = np.sqrt(L / a_max)
        v_peak = a_max * t_acc
        t_cruise = 0.0
        t_tot = 2.0 * t_acc
    else:
        # Trapezoidal: cruise exists
        d_cruise = L - 2*d_acc
        t_cruise = d_cruise / v_peak
        t_tot = 2.0 * t_acc + t_cruise

    # Discrete time grid (ensure final sample lands exactly)
    t = np.arange(0.0, t_tot, dt)
    if t.size == 0 or t[-1] < t_tot - 1e-12:
        t = np.concatenate([t, [t_tot]])

    s = np.empty_like(t)
    v = np.empty_like(t)
    a = np.empty_like(t)

    for i, ti in enumerate(t):
        if ti <= t_acc + 1e-12:
            # Accel phase
            a[i] = a_max
            v[i] = a_max * min(ti, t_acc)
            s[i] = 0.5 * a_max * min(ti, t_acc)**2
        elif ti <= t_acc + t_cruise + 1e-12:
            # Cruise
            tc = ti - t_acc
            a[i] = 0.0
            v[i] = v_peak
            s[i] = d_acc + v_peak * min(tc, t_cruise)
        else:
            # Decel
            td = ti - (t_acc + t_cruise)
            a[i] = -a_max
            v[i] = max(0.0, v_peak - a_max * td)
            s[i] = d_acc + (v_peak * t_cruise if t_cruise > 0 else 0.0) + v_peak * td - 0.5 * a_max * td**2

    # Clamp final sample exactly
    s[-1] = L
    v[-1] = 0.0
    a[-1] = 0.0
    return t, s, v, a

def trapezoid_sync(L, v_max, a_max, dt, t_sync=None):
    """
    Build a trapezoid/triangle profile for distance L.
    If t_sync is None → minimal-time profile under (v_max,a_max).
    If t_sync is given (>= minimal time) → compute v_peak to match t_sync.
    """
    if L <= 0:
        z = np.array([0.0])
        return z, z, z, z

    t_min = trapezoid_min_time(L, v_max, a_max)
    if (t_sync is None) or (t_sync <= t_min + 1e-12):
        # Minimal time
        v_peak = min(v_max, np.sqrt(max(L*a_max, 0.0)))
        return trapezoid_from_vpeak(L, a_max, v_peak, dt)
    else:
        # Stretch to match t_sync. Solve for v_peak, then clamp to v_max if needed and recompute.
        v_peak = solve_vpeak_for_time(L, a_max, t_sync)
        v_peak = min(v_peak, v_max)
        # If clamped, the resulting total time will be >= requested t_sync (still OK for synchronization).
        return trapezoid_from_vpeak(L, a_max, v_peak, dt)

# =============================================================================
# 3D SEGMENT BUILDER (translation + quaternion rotation, synchronized)
# =============================================================================
def build_segment_3d(p0, q0, p1, q1, dt, max_vel, max_acc, max_omg, max_alpha):
    """
    Build one synchronized segment from (p0,q0) → (p1,q1).
    - p*: (3,) position
    - q*: (4,) quaternion [w,x,y,z]
    Returns a dict with arrays sampled at 'dt':
      t, p (Nx3), q (Nx4), v_lin (Nx3), a_lin (Nx3), omega (Nx3), alpha (Nx3)
    """
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    R0 = q2R(q0)

    # Translation distance and direction
    d = p1 - p0
    L = float(np.linalg.norm(d))
    u = d / L if L > 1e-12 else np.array([1.0,0.0,0.0])

    # Relative rotation from q0 to q1 via R0^T R1
    R0 = q2R(q0)
    R1 = q2R(q1)
    R_rel = R0.T @ R1
    k, theta = R2rot(R_rel)     # axis k, angle theta>=0
    theta = _angle_wrap(theta)  # shortest angle in (-pi,pi]
    sgn = 1.0
    if theta < 0:
        theta = -theta
        sgn = -1.0
    if theta < 1e-12:
        k = np.array([1.0,0.0,0.0])  # arbitrary axis

    # Minimal times
    t_lin_min = trapezoid_min_time(L, max_vel, max_acc) if L > 0 else 0.0
    t_rot_min = trapezoid_min_time(theta, max_omg, max_alpha) if theta > 0 else 0.0
    t_sync = max(t_lin_min, t_rot_min, 0.0)

    # Build both profiles, stretched to t_sync
    t_lin, s_lin, v_lin, a_lin = trapezoid_sync(L, max_vel, max_acc, dt, t_sync if L>0 else None)
    t_rot, s_rot, omg, alp     = trapezoid_sync(theta, max_omg, max_alpha, dt, t_sync if theta>0 else None)

    # Use a common time vector (pick the longer; ensure same final time)
    T = t_lin if t_lin[-1] >= t_rot[-1] else t_rot
    # Re-index the shorter one to match T if needed (simple pad by last value; fine since both end at rest)
    def _pad_to(Tref, t, arrs):
        if np.isclose(Tref[-1], t[-1]):
            return arrs
        # last sample duplication to match size
        pad_n = len(Tref) - len(t)
        if pad_n <= 0:
            return [a[:len(Tref)] for a in arrs]
        return [np.concatenate([a, np.repeat(a[-1], pad_n)]) for a in arrs]

    if len(t_lin) != len(T):
        s_lin, v_lin, a_lin = _pad_to(T, t_lin, [s_lin, v_lin, a_lin])
        t_lin = T
    if len(t_rot) != len(T):
        s_rot, omg, alp = _pad_to(T, t_rot, [s_rot, omg, alp])
        t_rot = T

    # Sample positions & quats along the profiles
    P = p0[None,:] + s_lin[:,None] * u[None,:] if L>0 else np.repeat(p0[None,:], len(T), axis=0)

    # Rotation progress: angle(t) = sgn * s_rot(t); quaternion delta
    Q = np.zeros((len(T), 4))
    for i, ang in enumerate(sgn * s_rot):
        R_delta = rot(k, ang)
        Q[i] = R2q(R0 @ R_delta)
    if theta <= 1e-12:
        Q[:] = q0

    # Linear vel/acc in world along u
    Vlin = (v_lin[:,None] * u[None,:]) if L>1e-12 else np.zeros((len(T),3))
    Alin = (a_lin[:,None] * u[None,:]) if L>1e-12 else np.zeros((len(T),3))

    # Angular vel/acc vectors in world (about k from the start frame). Use start/world axis k.
    Omega = (sgn * omg)[:,None] * k[None,:] if theta>1e-12 else np.zeros((len(T),3))
    Alpha = (sgn * alp)[:,None] * k[None,:] if theta>1e-12 else np.zeros((len(T),3))

    # Ensure exact terminal state
    P[-1] = p1
    Q[-1] = q1
    Vlin[-1] = 0.0
    Alin[-1] = 0.0
    Omega[-1] = 0.0
    Alpha[-1] = 0.0

    return dict(t=T, p=P, q=Q, v_lin=Vlin, a_lin=Alin, omega=Omega, alpha=Alpha)

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
waypoints = [np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])] # start at origin with no rotation
# move to 1m in x direction
waypoints.append(np.array([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
# move to 0.3m in y direction
waypoints.append(np.array([0.3, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
# move to -0.3m in y direction
waypoints.append(np.array([-0.3, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
# move to 0m in y direction
waypoints.append(np.array([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
# move to 0.4m in z direction
waypoints.append(np.array([0.0, 1.0, 0.4, 1.0, 0.0, 0.0, 0.0]))
# rotate 15 degrees around z axis
rot_quat = rot2q(np.array([0,0,1]), np.radians(10))
waypoints.append(np.array([0.0, 1.0, 0.4, rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]]))
# rotate -15 degrees around z axis
rot_quat = rot2q(np.array([0,0,1]), np.radians(-10))
waypoints.append(np.array([0.0, 1.0, 0.4, rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]]))
# rotate back to 0 degrees around z axis
waypoints.append(np.array([0.0, 1.0, 0.4, 1.0, 0.0, 0.0, 0.0]))
# move back 0.5m in x direction
waypoints.append(np.array([0.0, 0.5, 0.4, 1.0, 0.0, 0.0, 0.0]))
# move back to 0.0m in z direction
waypoints.append(np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0]))
# move back to starting point
waypoints.append(np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))

# convert the waypoints into trajectory
pub_rate = 200 # Hz
max_vel = 0.08 # m/s
max_acc = 0.2 # m/s^2
max_omg = np.deg2rad(5) # rad/s
max_alpha = np.deg2rad(7.5) # rad/s^2

# Build trajectory
traj = generate_trajectory_3d(waypoints, pub_rate, max_vel, max_acc, max_omg, max_alpha)

# save it to a csv file with x,y,z,qx,qy,qz,qw
np.savetxt("simple_swarm_trajectory.csv", np.hstack((traj['p'], np.roll(traj['q'],3,axis=1))), delimiter=",", header="x,y,z,qw,qx,qy,qz", comments="")

# plot x y z position
import matplotlib.pyplot as plt
plt.figure()
plt.plot(traj['t'], traj['p'][:,0], label='x')
plt.plot(traj['t'], traj['p'][:,1], label='y')
plt.plot(traj['t'], traj['p'][:,2], label='z')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.title('Position vs Time')
plt.grid()
plt.show()

# plot qw, qx, qy, qz
plt.figure()
plt.plot(traj['t'], traj['q'][:,0], label='qw')
plt.plot(traj['t'], traj['q'][:,1], label='qx')
plt.plot(traj['t'], traj['q'][:,2], label='qy')
plt.plot(traj['t'], traj['q'][:,3], label='qz')
plt.xlabel('Time (s)')
plt.ylabel('Quaternion')
plt.legend()
plt.title('Quaternion vs Time')
plt.grid()
plt.show()