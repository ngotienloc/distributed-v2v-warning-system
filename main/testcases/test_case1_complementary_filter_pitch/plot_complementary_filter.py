import argparse
import math
import time

import matplotlib.pyplot as plt
import serial


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) < 9:
        return None
    try:
        return {
            "t": float(parts[0]),
            "ax": float(parts[1]),
            "ay": float(parts[2]),
            "az": float(parts[3]),
            "gx": float(parts[4]),
            "gy": float(parts[5]),
            "gz": float(parts[6]),
            "pitch": float(parts[7]),
            "roll": float(parts[8]),
        }
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to capture. 0 = until Ctrl+C")
    parser.add_argument("--axis", choices=["pitch", "roll"], default="pitch")
    parser.add_argument("--save", default=None, help="Save plot to PNG path")
    args = parser.parse_args()

    times = []
    ax = []
    ay = []
    az = []
    gx = []
    gy = []
    gz = []
    pitch = []
    roll = []

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        start = time.time()
        try:
            while True:
                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break
                line = ser.readline().decode("utf-8", errors="ignore")
                data = parse_line(line)
                if not data:
                    continue
                times.append(data["t"])
                ax.append(data["ax"])
                ay.append(data["ay"])
                az.append(data["az"])
                gx.append(data["gx"])
                gy.append(data["gy"])
                gz.append(data["gz"])
                pitch.append(data["pitch"])
                roll.append(data["roll"])
        except KeyboardInterrupt:
            pass

    if not times:
        print("No samples received. Check UART output format and port.")
        return

    t0 = times[0]
    times = [t - t0 for t in times]

    if args.axis == "pitch":
        accel_angle = [
            math.degrees(math.atan2(-ax[i], math.sqrt(ay[i] * ay[i] + az[i] * az[i])))
            for i in range(len(times))
        ]
        gyro_axis = gy
        filt_angle = pitch
        axis_label = "Pitch"
    else:
        accel_angle = [math.degrees(math.atan2(ay[i], az[i])) for i in range(len(times))]
        gyro_axis = gx
        filt_angle = roll
        axis_label = "Roll"

    gyro_angle = [0.0]
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        gyro_angle.append(gyro_angle[-1] + gyro_axis[i - 1] * dt)
    gyro_angle = [math.degrees(a) for a in gyro_angle]
    gyro_deg_s = [math.degrees(v) for v in gyro_axis]

    ax_g = [v / 9.81 for v in ax]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(times, ax_g, color="red")
    axes[0].set_ylabel("X-Acc (g)")
    axes[0].grid(True)

    axes[1].plot(times, gyro_deg_s, color="blue")
    axes[1].set_ylabel("Gyro (deg/s)")
    axes[1].grid(True)

    axes[2].plot(times, gyro_angle, label="Gyro Estimate")
    axes[2].plot(times, accel_angle, label="Accel Estimate")
    axes[2].plot(times, filt_angle, label="Filtered Estimate")
    axes[2].set_ylabel(f"{axis_label} (deg)")
    axes[2].set_xlabel("Time (sec)")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
