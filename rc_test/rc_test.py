import argparse
import json
import math
import time

import paho.mqtt.client as mqtt


def parse_args():
    parser = argparse.ArgumentParser(description="RC MQTT test publisher")
    parser.add_argument("--broker", default="192.168.100.10")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic-pose", default="wiserisk/p1/pose")
    parser.add_argument("--topic-goal", default="wiserisk/rc/goal")
    parser.add_argument("--topic-safety", default="wiserisk/rc/safety")
    parser.add_argument(
        "--dir",
        choices=["up", "down", "left", "right"],
        default="left",
        help="goal direction from origin",
    )
    parser.add_argument("--dist", type=float, default=2.0, help="goal distance (m)")
    parser.add_argument(
        "--mode",
        choices=["move", "fixed"],
        default="move",
        help="move: pose follows goal, fixed: pose stays fixed for turn test",
    )
    parser.add_argument("--x0", type=float, default=0.0, help="initial pose x")
    parser.add_argument("--y0", type=float, default=0.0, help="initial pose y")
    parser.add_argument("--yaw0", type=float, default=0.0, help="initial pose yaw(rad)")
    return parser.parse_args()


def direction_to_goal(direction, dist):
    if direction == "up":
        return dist, 0.0
    if direction == "down":
        return -dist, 0.0
    if direction == "left":
        return 0.0, dist
    return 0.0, -dist


def main():
    args = parse_args()
    client = mqtt.Client()
    client.connect(args.broker, args.port, 60)
    client.loop_start()

    x, y, yaw = args.x0, args.y0, args.yaw0
    goal_x, goal_y = direction_to_goal(args.dir, args.dist)

    client.publish(
        args.topic_goal,
        json.dumps(
            {"x": goal_x, "y": goal_y, "frame": "world", "ts_ms": int(time.time() * 1000)}
        ),
        qos=1,
    )
    client.publish(
        args.topic_safety,
        json.dumps({"estop": False, "obstacle_stop": False, "planner_fail": False}),
        qos=1,
    )

    print(f"Goal dir={args.dir} -> ({goal_x:.2f}, {goal_y:.2f}), mode={args.mode}")

    try:
        while True:
            dx = goal_x - x
            dy = goal_y - y
            dist = math.sqrt(dx**2 + dy**2)

            if args.mode == "move" and dist > 0.15:
                step = 0.05
                x += step * (dx / dist)
                y += step * (dy / dist)
                yaw = math.atan2(dy, dx)

            ts_ms = int(time.time() * 1000)
            pose = {"x": x, "y": y, "yaw": yaw, "frame": "world", "ts_ms": ts_ms}
            client.publish(args.topic_pose, json.dumps(pose), qos=1)
            print(f"POSE -> x={x:.2f} y={y:.2f} yaw={yaw:.2f} dist={dist:.2f}")

            if args.mode == "move" and dist <= 0.15:
                print("Goal REACHED")
                break

            time.sleep(0.1)  # 10Hz
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
