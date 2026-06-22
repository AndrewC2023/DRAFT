"""Plot the QE missile scenario logs.

The script is intentionally simple: it loads every scenario log in the local
logs folder and writes a few PNG figures into the results folder.
"""

import json
import math
import os
from pathlib import Path

script_directory = Path(__file__).resolve().parent
os.environ.setdefault("MPLCONFIGDIR", str(script_directory / ".matplotlib"))

import matplotlib.pyplot as plt


log_directory = script_directory / "logs"
results_directory = script_directory / "results"
results_directory.mkdir(exist_ok=True)

state_titles = [
    "Airspeed, $v_\\infty$",
    "Heading angle, $\\sigma$",
    "Flight-path angle, $\\gamma$",
]
state_units = ["m/s", "deg", "deg"]
input_titles = [
    "Thrust",
    "Turn thrust-vector angle, $\\delta_t$",
    "Dive thrust-vector angle, $\\delta_d$",
]
input_units = ["N", "deg", "deg"]
default_max_thrust = 500000.0


def set_reference_centered_bounds(axis, actual_values, reference_values, margin):
    lower = min(reference_values) - margin
    upper = max(reference_values) + margin

    # If a controller performs poorly, keep the full response visible while
    # still centering the plot around the requested trajectory when it tracks.
    lower = min(lower, min(actual_values) - 0.25 * margin)
    upper = max(upper, max(actual_values) + 0.25 * margin)

    if upper - lower < 2.0 * margin:
        center = 0.5 * (upper + lower)
        lower = center - margin
        upper = center + margin

    axis.set_ylim(lower, upper)


def load_trial(path):
    with path.open("r", encoding="utf-8") as log_file:
        return json.load(log_file)


def as_angle_if_needed(value, index):
    if index > 0:
        return math.degrees(value)
    return value


def get_reference_series(trial, state_index):
    if "referenceOutput" in trial["samples"][0]:
        values = [
            sample["referenceOutput"][state_index]
            for sample in trial["samples"]
        ]
    elif "referenceState" in trial:
        values = [
            trial["referenceState"][state_index]
            for _ in trial["samples"]
        ]
    else:
        values = [
            trial["finalReferenceState"][state_index]
            for _ in trial["samples"]
        ]

    return [as_angle_if_needed(value, state_index) for value in values]


def get_input_reference(trial, input_index):
    if "referenceInput" not in trial:
        return None

    value = trial["referenceInput"][input_index]
    return as_angle_if_needed(value, input_index)


def clean_title(text):
    if text == "scenario2_boost_to_space":
        return "Scenario 2 Boost Upwards"
    return text.replace("_", " ").title()


def plot_response(trial, figure_path):
    times = [sample["time"] for sample in trial["samples"]]
    states = [sample["state"] for sample in trial["samples"]]
    commanded_inputs = [
        sample["commandedInput"] for sample in trial["samples"]
    ]
    achieved_inputs = [
        sample["achievedInput"] for sample in trial["samples"]
    ]

    figure, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

    for index in range(3):
        state_values = [
            as_angle_if_needed(state[index], index) for state in states
        ]
        reference_values = get_reference_series(trial, index)
        commanded_values = [
            as_angle_if_needed(command[index], index)
            for command in commanded_inputs
        ]
        achieved_values = [
            as_angle_if_needed(input_state[index], index)
            for input_state in achieved_inputs
        ]

        axes[0, index].plot(
            times,
            state_values,
            color="tab:blue",
            label="Actual state")
        axes[0, index].plot(
            times,
            reference_values,
            color="black",
            linestyle="--",
            label="Reference")

        axes[1, index].plot(
            times,
            commanded_values,
            color="tab:orange",
            linestyle="--",
            label="Commanded input")
        axes[1, index].plot(
            times,
            achieved_values,
            color="tab:green",
            label="Achieved actuator")

        input_reference = get_input_reference(trial, index)
        if input_reference is not None:
            axes[1, index].axhline(
                input_reference,
                color="black",
                linestyle=":",
                label="Linearization input")

        if index == 1:
            set_reference_centered_bounds(
                axes[0, index],
                state_values,
                reference_values,
                margin=5.0)
            axes[1, index].set_ylim(-45.0, 45.0)
        if index == 2:
            set_reference_centered_bounds(
                axes[0, index],
                state_values,
                reference_values,
                margin=5.0)
            axes[1, index].set_ylim(-80.0, 80.0)
        if index == 0:
            if trial.get("scenarioName") == "scenario1_level_heading":
                reference_center = reference_values[-1]
                axes[0, index].set_ylim(
                    reference_center - 20.0,
                    reference_center + 20.0)

            max_thrust = max(
                default_max_thrust,
                max(commanded_values),
                max(achieved_values))
            axes[1, index].set_ylim(0.0, 1.05 * max_thrust)

        axes[0, index].set_title(state_titles[index])
        axes[0, index].set_ylabel(state_units[index])
        axes[1, index].set_title(input_titles[index])
        axes[1, index].set_ylabel(input_units[index])
        axes[1, index].set_xlabel("Time (s)")

        axes[0, index].grid(True)
        axes[1, index].grid(True)
        axes[0, index].ticklabel_format(useOffset=False, style="plain")
        axes[1, index].ticklabel_format(useOffset=False, style="plain")
        axes[0, index].legend()
        axes[1, index].legend()

    scenario_title = clean_title(trial.get("scenarioName", "scenario"))
    controller = trial.get("controller", "controller")
    figure.suptitle(f"{scenario_title} - {controller}")
    figure.tight_layout()
    figure.savefig(figure_path, dpi=200)
    plt.close(figure)


def plot_mass(trial, figure_path):
    times = [sample["time"] for sample in trial["samples"]]
    masses = [sample["state"][3] for sample in trial["samples"]]

    figure, axis = plt.subplots(figsize=(8, 5))
    axis.plot(times, masses, color="tab:purple")
    axis.set_title(f"{clean_title(trial['scenarioName'])} - Mass")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Mass (kg)")
    axis.grid(True)
    axis.ticklabel_format(useOffset=False, style="plain")
    figure.tight_layout()
    figure.savefig(figure_path, dpi=200)
    plt.close(figure)


def plot_altitude(trial, figure_path):
    times = [sample["time"] for sample in trial["samples"]]
    altitudes = [-sample["state"][6] for sample in trial["samples"]]

    figure, axis = plt.subplots(figsize=(8, 5))
    axis.plot(times, altitudes, color="tab:cyan")
    axis.set_title(f"{clean_title(trial['scenarioName'])} - Altitude")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Altitude (m)")
    axis.grid(True)
    axis.ticklabel_format(useOffset=False, style="plain")
    figure.tight_layout()
    figure.savefig(figure_path, dpi=200)
    plt.close(figure)


def plot_xy(trial, figure_path):
    states = [sample["state"] for sample in trial["samples"]]
    north = [state[4] for state in states]
    east = [state[5] for state in states]

    figure, axis = plt.subplots(figsize=(7, 7))
    axis.plot(east, north, color="tab:blue")
    axis.scatter(east[0], north[0], color="tab:green", label="Start")
    axis.scatter(east[-1], north[-1], color="tab:red", label="End")
    axis.set_title(f"{clean_title(trial['scenarioName'])} - XY Projection")
    axis.set_xlabel("East position (m)")
    axis.set_ylabel("North position (m)")
    axis.axis("equal")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(figure_path, dpi=200)
    plt.close(figure)


def plot_trajectory_3d(trial, figure_path):
    states = [sample["state"] for sample in trial["samples"]]
    north = [state[4] for state in states]
    east = [state[5] for state in states]
    altitude = [-state[6] for state in states]

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(east, north, altitude, color="tab:blue")
    axis.scatter(east[0], north[0], altitude[0], color="tab:green", label="Start")
    axis.scatter(east[-1], north[-1], altitude[-1], color="tab:red", label="End")
    axis.set_title(f"{clean_title(trial['scenarioName'])} - 3D Trajectory")
    axis.set_xlabel("East position (m)")
    axis.set_ylabel("North position (m)")
    axis.set_zlabel("Altitude (m)")
    axis.legend()
    figure.tight_layout()
    figure.savefig(figure_path, dpi=200)
    plt.close(figure)


log_paths = sorted(log_directory.glob("scenario*.json"))
if not log_paths:
    raise FileNotFoundError(f"No scenario logs found in {log_directory}")

for log_path in log_paths:
    trial = load_trial(log_path)
    figure_stem = log_path.stem

    plot_response(trial, results_directory / f"{figure_stem}_response.png")
    plot_mass(trial, results_directory / f"{figure_stem}_mass.png")
    plot_altitude(trial, results_directory / f"{figure_stem}_altitude.png")
    plot_xy(trial, results_directory / f"{figure_stem}_xy.png")
    plot_trajectory_3d(
        trial,
        results_directory / f"{figure_stem}_trajectory_3d.png")

    print(f"Saved figures for {figure_stem}")
