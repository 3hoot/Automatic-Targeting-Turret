import os
import sys
from contextlib import redirect_stdout
from typing import cast

import control
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal

OUTPUT_TO_FILE = True
ALLOW_OVERWRITE = False
OUTPUT_FILENAME = "out/adrc_synthesis_results_ax0.txt"
SHOW_PLOT = True
PLOT_FILENAME = "out/adrc_synthesis_ax0.png"
NOISE_STD_DEG = 0.0005   # sensor noise for the time simulation (set e.g. 0.005 to test)

SAMPLING_TIME_S = 1e-2  # (10 ms)

# Zero-order hold discretization of a continuous-time state-space system
def c2d_zoh(A, B, C, D, Ts):
    (Ad, Bd, Cd, Dd, _) = cast(
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float],
        signal.cont2discrete((A, B, C, D), Ts, method="zoh"))
    return Ad, Bd, Cd, Dd

# Acceleration-limited S-curve trajectory generator
def s_curve(x_start, x_end, v_max, a_max, Ts):    
    # To minimize time spent traveling, we'd like to use maximum acceleration as much as possible
    # but we also need to respect the maximum velocity.
    # The S-curve profile consists of three phases: acceleration, constant velocity, and deceleration.
    
    # We could worry about jerk, but with ADRC, we can just limit the acceleration and let the controller handle the rest.

    t_v_max = v_max / a_max                     # Time to reach maximum velocity
    x_accel = 0.5 * a_max * t_v_max**2          # Distance covered during acceleration phase
    x_decel = x_accel                           # Symmetric deceleration phase
    x_dist = x_end - x_start                    # Total distance to travel
    x_const_vel = x_dist - x_accel - x_decel    # Distance covered during constant velocity phase

    if x_const_vel <= 0: 
        # If the distance is too short, we need to shorten the acceleration phase and skip the constant velocity phase.
        x_accel = x_dist / 2
        t_accel = np.sqrt(2 * x_accel / a_max)
        t_decel = t_accel
        t_const_vel = 0
    else:
        # We're able to reach maximum velocity and maintain it for a while before decelerating.
        t_accel = t_v_max
        t_decel = t_v_max
        t_const_vel = x_const_vel / v_max

    # Total time for the trajectory
    t_total = t_accel + t_const_vel + t_decel
    n_samples = int(np.ceil(t_total / Ts)) + 1  # +1 to include the final point
    x_traj = np.zeros(n_samples)
    v_traj = np.zeros(n_samples) # We hold this as well for derivative term of the ADRC 
    for i in range(n_samples):
        t = i * Ts
        if t < t_accel:
            # Acceleration phase
            x_traj[i] = x_start + 0.5 * a_max * t**2
            v_traj[i] = a_max * t
        elif t < t_accel + t_const_vel:
            # Constant velocity phase
            x_traj[i] = x_start + x_accel + v_max * (t - t_accel)
            v_traj[i] = v_max
        elif t < t_total:
            # Deceleration phase
            t_decel_phase = t - (t_accel + t_const_vel)
            x_traj[i] = x_end - 0.5 * a_max * (t_decel - t_decel_phase)**2
            v_traj[i] = a_max * (t_decel - t_decel_phase)
        else:
            # After the trajectory is complete, hold the final position
            x_traj[i] = x_end
            v_traj[i] = 0.0

    return x_traj, v_traj, n_samples, t_total


def main():
    # Method references:
    # - Han, J. (2009), "From PID to Active Disturbance Rejection Control".
    # - Herbst, G. & Madonski, R. (2023), "Tuning and Implementation Variants
    #   of Discrete-Time ADRC".
    Ts = SAMPLING_TIME_S

    # Identified continuous-time model:  tau*y_ddot + y_dot = Kv*u
    Kv = 0.010636921289335   # Gain (deg/s per Hz)
    tau = 0.012944388107410  # Time constant (s)
    b0 = Kv / tau            # Input gain on y_ddot (deg/s^2 per Hz)

    # Tuning parameters
    # These need to be tuned below the Nyquist frequency of the system (pi/Ts ~ 314.159 rad/s).
    omega_c = 60.0              # Regulator bandwidth (rad/s)
    omega_o = 120.0             # Observer bandwidth (rad/s)
    k_eso = omega_o / omega_c   # Observer bandwidth scaling factor

    # ---------------- Observer tuning ---------------- #

    # Model-assisted ESO, states [y, y_dot, d_total]
    A = np.array([[0, 1, 0],
                  [0, -1 / tau, 1],
                  [0, 0, 0]])
    B = np.array([[0], [b0], [0]])
    C = np.array([[1, 0, 0]])
    D = np.array([[0]])
    Ad, Bd, Cd, _ = c2d_zoh(A, B, C, D, Ts)

    # Current-estimator form (Herbst & Madonski): Lp = Ad @ Lc
    observer_poles = np.full(3, np.exp(-k_eso * omega_c * Ts))
    Lp = control.acker(Ad.T, Cd.T, observer_poles).reshape(-1, 1)  # predictive gain
    L = np.linalg.solve(Ad, Lp)                                   # corrective gain

    # ESO matrices
    Ad_eso = Ad - L @ Cd @ Ad
    Bd_eso = Bd - L @ Cd @ Bd

    # Pole placement verification (should match observer_poles)
    obs_check = np.linalg.eigvals(Ad_eso)
    print("\nObserver design verification:")
    print(f"\tObserver poles ( desired ): {observer_poles}")
    print(f"\tObserver poles ( actual  ): {np.abs(obs_check)}")
    print(f"\tL = [{L[0,0]:.4e}, {L[1,0]:.4e}, {L[2,0]:.4e}]")
    print(f"\tAd_eso =\n{Ad_eso}")
    print(f"\tBd_eso =\n{Bd_eso}")

    # ---------------- Regulator tuning --------------- #

    # "Virtual" plant (disturbance state removed)
    Avp = np.array([[0, 1],
                    [0, -1 / tau]])
    Bvp = np.array([[0], [b0]])
    Cvp = np.array([[1, 0]])
    Dvp = np.array([[0]])
    Avpd, Bvpd, _, _ = c2d_zoh(Avp, Bvp, Cvp, Dvp, Ts)

    # K is in units of Hz/deg and Hz/(deg/s): it already contains 1/b0.
    # Because Bvp includes b0, the poles of (Avpd - Bvpd K) are exactly placed.
    regulator_poles = np.full(2, np.exp(-omega_c * Ts))
    K = control.acker(Avpd, Bvpd, regulator_poles).flatten()  # shape (2,)

    reg_check = np.linalg.eigvals(Avpd - Bvpd @ K.reshape(1, -1))
    print("\nRegulator design verification:")
    print(f"\tRegulator poles ( desired ): {regulator_poles}")
    print(f"\tRegulator poles ( actual  ): {np.abs(reg_check)}")
    print(f"\tK = [{K[0]:.4e}, {K[1]:.4e}, 1/b0 = {1/b0:.4e}]")

    # ------ Simulation against the state model ------- #

    def physical_plant(x, u, d_ext):
        y, y_dot = x
        y_ddot = -(1.0 / tau) * y_dot + b0 * u + d_ext
        return np.array([y_dot, y_ddot])

    def rk4_step(x, u, d_ext, h):
        k1 = physical_plant(x, u, d_ext)
        k2 = physical_plant(x + 0.5 * h * k1, u, d_ext)
        k3 = physical_plant(x + 0.5 * h * k2, u, d_ext)
        k4 = physical_plant(x + h * k3, u, d_ext)
        return x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    t_sim = 10.0
    u_max = 20000.0    # Hz   (hardware limit)
    du_max = 20000.0   # Hz/s (servo slew limit)
    du_step = du_max * Ts

    steps = int(round(t_sim / Ts))
    time = np.arange(steps) * Ts

    # Maxium velocity and acceleration for the trajectory feedforward (deg/s and deg/s^2)
    v_max = 0.8 * Kv * u_max                # deg/s, maximum achievable velocity, correction factor 0.8 to avoid saturation (20% margin)
    a_max_act = Kv * du_max                 # deg/s^2, maximum achievable acceleration (actuator limit)
    a_max_mag = b0 * u_max                  # deg/s^2, maximum achievable acceleration (magnitude limit)
    a_max = 0.8 * min(a_max_act, a_max_mag) # correction factor 0.8 to avoid saturation (20% margin)

    # Move distance for the trajectory (deg)
    move = 30.0
    k_traj = int(round(2.0 / Ts))  # Start the trajectory after 2 seconds
    r_traj = np.zeros(steps)
    v_traj = np.zeros(steps)

    # Generate the S-curve trajectory for the reference position and velocity
    r_profile, v_profile, n_samples, t_total = s_curve(0.0, move, v_max, a_max, Ts)
    r_traj[k_traj:k_traj+n_samples] = r_profile
    v_traj[k_traj:k_traj+n_samples] = v_profile

    # After the trajectory is complete, hold the final position
    r_traj[k_traj+n_samples:] = move

    # Disturbance in deg/s^2. It must be compensable by the actuator:
    # steady-state needed |u| = |d|*tau/Kv  (-30 deg/s^2 -> ~37 Hz).
    k_dist = int(round(5.0 / Ts))
    d_ext_traj = np.zeros(steps)
    d_ext_traj[k_dist:] = -30.0  # Step disturbance after 5 seconds

    x_plant = np.zeros(2)           # [y, y_dot]
    x_estimate = np.zeros((3, 1))   # ESO prediction of x[k] from data up to k-1
    u_prev = 0.0

    y_log = np.zeros(steps)
    x_estimate_log = np.zeros((3, steps))
    u_log = np.zeros(steps)

    rng = np.random.default_rng(0)
    for k in range(steps):
        
        # 0) Set r as the reference trajectory
        r = r_traj[k]
        dr = v_traj[k] 

        # 1) Measure y[k]
        y_meas = x_plant[0] + NOISE_STD_DEG * rng.standard_normal()

        # 2) ESO state estimation:
        x_estimate = Ad_eso @ x_estimate + Bd_eso * u_prev + L * y_meas

        # 3) Control law: state feedback + disturbance compensation
        K_full = np.array([K[0], K[1], 1.0 / b0])  # add disturbance compensation term
        u_calc = K[0] * r + K[1] * dr - K_full @ x_estimate    # already includes 1/b0 for the disturbance compensation (look up Bvp)

        # 4) Limiter: rate limit first, then magnitude limit
        u_rate_limited = np.clip(u_calc, u_prev - du_step, u_prev + du_step)
        u_applied = (np.clip(u_rate_limited, -u_max, u_max)).item()  # convert to scalar
        u_prev = u_applied

        # 5) Log (all quantities belong to sample k)
        y_log[k], x_estimate_log[:, k], u_log[k] = x_plant[0], x_estimate.flatten(), u_applied # x_plant[0] instead of y_meas to not log the noise

        # 6) Advance plant (ZOH over [k, k+1]) and ESO prediction
        x_plant = rk4_step(x_plant, u_applied, d_ext_traj[k], Ts)

    # Simple metrics
    print("\nSimulation summary:")
    print(f"\tPeak output after step: {y_log.max():.3f} deg")
    print(f"\tOvershoot: {100.0 * (y_log.max() - move) / move:.3f} %")
    print(f"\tPeak error: {np.abs(r_traj - y_log).max():.3f} deg")
    print(f"\tFinal disturbance estimate (true = {d_ext_traj[-1]}): {x_estimate_log[2, -1]:.4f}")

    # Plotting
    fig, axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True)

    axs[0][0].plot(time, r_traj, "r--", label="Reference $r(t)$")
    axs[0][0].plot(time, y_log, "b-", label="Plant Output $y(t)$")
    axs[0][0].plot(time, x_estimate_log[0, :], "g:", label="ESO Estimate $z_1(t)$")
    axs[0][0].set_ylabel("Position (deg)")
    axs[0][0].set_title("MA-ADRC Closed-Loop Performance")
    axs[0][0].legend(loc="lower right")
    axs[0][0].grid(True)

    axs[0][1].plot(time, d_ext_traj, "r--", label="Actual Load $d_{ext}(t)$")
    axs[0][1].plot(time, x_estimate_log[2, :], "m-", label="Estimated Disturbance $z_3(t)$")
    axs[0][1].set_ylabel("Disturbance (deg/s²)")
    axs[0][1].set_title("MA-ESO Extended State Tracking")
    axs[0][1].legend(loc="lower right")
    axs[0][1].grid(True)

    axs[1][0].plot(time, u_log, "k-", label="Control Signal $u(t)$")
    axs[1][0].set_xlabel("Time (seconds)")
    axs[1][0].set_ylabel("Control Input $u$ (Hz)")
    axs[1][0].legend(loc="lower right")
    axs[1][0].grid(True)

    e_track = r_traj - y_log
    axs[1][1].plot(time, e_track, "c-", label="Tracking Error $e(t)$")
    axs[1][1].set_xlabel("Time (seconds)")
    axs[1][1].set_ylabel("Error (deg)")
    axs[1][1].set_title("Tracking Error")
    axs[1][1].legend(loc="lower right")
    axs[1][1].grid(True)

    plt.tight_layout()
    if PLOT_FILENAME:
        os.makedirs(os.path.dirname(PLOT_FILENAME) or ".", exist_ok=True)
        fig.savefig(PLOT_FILENAME, dpi=150)
    if SHOW_PLOT:
        plt.show()


if __name__ == "__main__":
    if OUTPUT_TO_FILE:
        os.makedirs(os.path.dirname(OUTPUT_FILENAME) or ".", exist_ok=True)
        mode = "w" if ALLOW_OVERWRITE else "x"
        try:
            with open(OUTPUT_FILENAME, mode) as f, redirect_stdout(f):
                main()
        except FileExistsError:
            print(f"Error: The file '{OUTPUT_FILENAME}' already exists.")
            sys.exit(1)
    else:
        main()
