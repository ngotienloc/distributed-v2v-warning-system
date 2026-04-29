import argparse
import math
import time

import matplotlib.pyplot as plt
import serial


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) < 7:
        return None
    try:
        return {
            "t": float(parts[0]),
            "gz": float(parts[1]),
            "gps_heading": float(parts[2]),
            "gps_speed": float(parts[3]),
            "gps_valid": int(parts[4]),
            "heading_imu": float(parts[5]),
            "heading_fused": float(parts[6]),
        }
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to capture. 0 = until Ctrl+C")
    parser.add_argument("--save", default=None, help="Save plot to PNG path")
    args = parser.parse_args()

    times = []
    gz = []
    gps_heading = []
    gps_speed = []
    heading_imu = []
    heading_fused = []

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
                gz.append(data["gz"])
                gps_speed.append(data["gps_speed"])
                heading_imu.append(data["heading_imu"])
                heading_fused.append(data["heading_fused"])
                if data["gps_valid"]:
                    gps_heading.append(data["gps_heading"])
                else:
                    gps_heading.append(float("nan"))
        except KeyboardInterrupt:
            pass

    if not times:
        print("No samples received. Check UART output format and port.")
        return

    t0 = times[0]
    times = [t - t0 for t in times]

    gyro_deg_s = [math.degrees(v) for v in gz]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(times, gyro_deg_s, color="blue")
    axes[0].set_ylabel("Gyro Z (deg/s)")
    axes[0].grid(True)

    axes[1].plot(times, gps_speed, color="green")
    axes[1].set_ylabel("GPS Speed (m/s)")
    axes[1].grid(True)

    axes[2].plot(times, gps_heading, label="GPS Heading")
    axes[2].plot(times, heading_imu, label="IMU Heading")
    axes[2].plot(times, heading_fused, label="Fused Heading")
    axes[2].set_ylabel("Heading (deg)")
    axes[2].set_xlabel("Time (sec)")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
