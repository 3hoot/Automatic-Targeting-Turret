import os
import sys
import csv
from contextlib import redirect_stdout

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal
import scipy.optimize as optimize

INPUT_FILENAME = "log/log_data_4k_ax0.csv"
OUTPUT_TO_FILE = True
ALLOW_OVERWRITE = False
OUTPUT_FILENAME = "out/step_response_results_ax0.txt"
SHOW_PLOT = True
PLOT_FILENAME = "out/step_response_ax0.png"  # e.g. "out/step_response.png" to save instead of / as well as show

REGRESSION_WINDOW_DURATION_MS = 100


def rolling_steps_per_degree(positions: list[float], step_positions: np.ndarray, window_size: int):
    if window_size < 2:
        raise ValueError("window_size must be at least 2")
    if len(positions) != len(step_positions):
        raise ValueError("positions and step_positions must have the same length")

    estimates = np.full(len(positions), np.nan)
    for end in range(window_size, len(positions) + 1):
        position_window = np.asarray(positions[end - window_size:end])
        steps_window = step_positions[end - window_size:end]
        position_centered = position_window - position_window.mean()
        position_variance = np.sum(position_centered**2)

        if position_variance == 0:
            continue

        steps_centered = steps_window - steps_window.mean()
        estimates[end - 1] = np.sum(
            position_centered * steps_centered
        ) / position_variance

    return estimates


def main():

    with open(INPUT_FILENAME, mode="r") as csvfile:
        csv_reader = csv.DictReader(csvfile)
        timestamps = []
        currents = []
        voltages = []
        angles = []
        positions = []
        target_positions = []
        step_positions = []
        commanded_hzs = []
        applied_hzs = []

        for row in csv_reader:
            timestamps.append(float(row["timestamp_us"]))
            currents.append(float(row["current"]))
            voltages.append(float(row["voltage"]))
            angles.append(float(row["angle"]))
            positions.append(float(row["position"]))
            target_positions.append(float(row["target_position"]))
            step_positions.append(float(row["step_position"]))
            commanded_hzs.append(float(row["commanded_hz"]))
            applied_hzs.append(float(row["applied_hz"]))

    timestamps_ms = (np.array(timestamps) - np.min(timestamps)) / 1000.0  # Convert microseconds to milliseconds
    step_positions = np.array(step_positions) * -1  # Reverse the stepper positions to match the measured positions

    sampling_period_ms = np.mean(np.diff(timestamps_ms))
    sampling_period_deviation = np.std(np.diff(timestamps_ms))
    max_sampling_period = np.max(np.diff(timestamps_ms))
    min_sampling_period = np.min(np.diff(timestamps_ms))

    print("\nSampling period statistics:")
    print(f"\tMean: {sampling_period_ms:.2f} ms")
    print(f"\tMax: {max_sampling_period:.2f} ms")
    print(f"\tMin: {min_sampling_period:.2f} ms")
    print(f"\tStd Dev: {sampling_period_deviation:.6f} ms")

    sampling_frequency_hz = 1000.0 / sampling_period_ms if sampling_period_ms > 0 else float('inf')
    print(f"\nEstimated sampling frequency: {sampling_frequency_hz:.2f} Hz")

    # -------------- Postion discrepancy -------------- #

    norm_step_positions = [(p - step_positions[0])/(max(step_positions) - min(step_positions)) for p in step_positions]
    norm_positions = [(p - positions[0])/(max(positions) - min(positions)) for p in positions]
    positions_diff = [norm_step_positions[i] - norm_positions[i] for i in range(len(norm_positions))]

    print("\nPosition difference statistics:")
    print(f"\tMax: {max(positions_diff) * 100:.2f}%")
    print(f"\tAvg: {np.mean(np.abs(positions_diff)) * 100:.2f}%")
    print(f"\tStd Dev: {np.std(positions_diff) * 100:.4f}%")
    print(f"\tVariance: {np.var(positions_diff) * 100:.4f}%")

    # ------------- Motor to sensor ratio ------------- #

    total_step_delta = step_positions[-1] - step_positions[0]
    total_position_delta = positions[-1] - positions[0]
    total_steps_per_degree = total_step_delta / total_position_delta if total_position_delta != 0 else float('inf')
    regression_window_samples = int(REGRESSION_WINDOW_DURATION_MS / sampling_period_ms)
    rolling_steps_per_degree_values = rolling_steps_per_degree(
        positions,
        step_positions,
        regression_window_samples,
    )

    MICROSTEPPING = 32
    GEAR_RATIO = 96 / 20  # Change to 58 / 20 for upper axis
    STEPS_PER_REVOLUTION = 200  # Standard stepper motor has 200 steps per revolution
    expected_steps_per_degree = MICROSTEPPING * GEAR_RATIO * (STEPS_PER_REVOLUTION / 360)
    print("\nSteps per degree statistics:")
    print(f"\tTotal: {total_steps_per_degree:.10f}")
    print(f"\tExpected: {expected_steps_per_degree:.10f}")
    print(f"\tRelative error: {abs(total_steps_per_degree - expected_steps_per_degree) / expected_steps_per_degree * 100:.5f}%")

    # ------------------ State model ------------------ #

    # Fixing the timestamps to have a constant sampling period for the system identification
    timestamps_s = timestamps_ms / 1000.0
    sampling_period_s = sampling_period_ms / 1000.0
    t_uniform = np.arange(timestamps_s[0], timestamps_s[-1], sampling_period_s)
    u_uniform = np.interp(t_uniform, timestamps_s, applied_hzs)
    y_uniform = np.interp(t_uniform, timestamps_s, positions)

    # Wrapper for the motor system response
    def motor_step_response(t, tau, Kv):
        A = np.array([[0.0, 1.0],
                    [0.0, -1.0 / tau]])
        B = np.array([[0.0],
                    [Kv / tau]])
        C = np.array([[1.0, 0.0]])
        D = np.array([[0.0]])

        sys = signal.StateSpace(A, B, C, D)
        X0 = [y_uniform[0], 0.0]  # Initial state: [position, velocity]
        _, y_out, _ = signal.lsim(sys, U=u_uniform, T=t, X0=X0)

        return y_out

    initial_params = [0.05, 1.0e-2]            # tau (s), Kv (deg/s per Hz)
    bounds = ([1e-4, 0.0], [5.0, np.inf])      # wide enough that tau isn't pinned

    y_fitted = None
    try:
        fitted_params, _ = optimize.curve_fit(motor_step_response, t_uniform, y_uniform, p0=initial_params, bounds=bounds)
        tau_est, Kv_est = fitted_params

        print("\nEstimated state model parameters:")
        print(f"\tTau: {tau_est:.15f}")
        print(f"\tKv: {Kv_est:.15f}")

        # Generate fitted response
        y_fitted = motor_step_response(t_uniform, tau_est, Kv_est)

        RMSE = np.sqrt(np.mean((y_uniform - y_fitted) ** 2))
        NRMSE = RMSE / (np.max(y_uniform) - np.min(y_uniform)) if (np.max(y_uniform) - np.min(y_uniform)) != 0 else float('inf')
        print(f"\tRMSE: {RMSE:.6f}")
        print(f"\tNRMSE: {NRMSE:.6f}%")

    except Exception as e:
        print(f"Error during parameter estimation: {e}")

    # Plotting
    fig, axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True)

    axs[0][0].plot(timestamps_ms, norm_step_positions, label="Stepper Position", marker='o', markersize=3)
    axs[0][0].plot(timestamps_ms, norm_positions, label="Measured Position", marker='o', markersize=3)
    axs[0][0].set_title("Step Response (Normalized)")
    axs[0][0].set_ylabel("Position")
    axs[0][0].legend(loc="best")
    axs[0][0].grid(True)

    axs[0][1].plot(timestamps_ms, rolling_steps_per_degree_values, label=f"Rolling regression ({regression_window_samples} samples)", marker='o', markersize=3)
    axs[0][1].axhline(expected_steps_per_degree, color="r", linestyle="--", label="Expected steps per degree")
    axs[0][1].axhline(total_steps_per_degree, color="g", linestyle="--", label="Total steps per degree")
    axs[0][1].set_title("Rolling Steps-per-Degree Regression")
    axs[0][1].set_ylabel("Steps per Degree")
    axs[0][1].legend(loc="best")
    axs[0][1].grid(True)

    axs[1][0].plot(timestamps_ms, positions_diff, label="Position Difference", marker='o', markersize=3)
    axs[1][0].set_title("Position Difference (Normalized)")
    axs[1][0].set_xlabel("Time (ms)")
    axs[1][0].set_ylabel("Difference")
    axs[1][0].legend(loc="best")
    axs[1][0].grid(True)

    axs[1][1].plot(timestamps_ms, positions, label="Measured Position", marker='o', markersize=3)
    axs[1][1].plot(t_uniform * 1000, y_fitted, label="Fitted Response", linestyle='--')
    axs[1][1].set_title("Measured vs Fitted Step Response")
    axs[1][1].set_xlabel("Time (ms)")
    axs[1][1].set_ylabel("Position")
    axs[1][1].legend(loc="best")
    axs[1][1].grid(True)

    plt.tight_layout()
    if PLOT_FILENAME:
        os.makedirs(os.path.dirname(PLOT_FILENAME) or ".", exist_ok=True)
        fig.savefig(PLOT_FILENAME, dpi=150)
    if SHOW_PLOT:
        plt.show()


if __name__ == "__main__":
    mode = "w" if ALLOW_OVERWRITE else "x"
    if OUTPUT_TO_FILE:
        try:
            with open(OUTPUT_FILENAME, mode) as f:
                with redirect_stdout(f):
                    main()
        except FileExistsError:
            print(f"Error: The file '{OUTPUT_FILENAME}' already exists.")
            sys.exit(1)
    else:
        main()
