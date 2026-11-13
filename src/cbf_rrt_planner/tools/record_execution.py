#!/usr/bin/env python3
"""
Records what the robot actually drove for every path of planner_node (README_EXPERIMENTS.md, §5).

    python3 src/cbf_rrt_planner/tools/record_execution.py --label cbf [--out DIR] [--base-frame base_link]
        [--rate 20] [--ros-args -p use_sim_time:=true]

Every new /planned_path starts run n and is written to <label>_<n>_plan.csv (x,y). While the
FollowPath goal is executing (/follow_path/_action/status), the TF map -> base_link is sampled and
written to <label>_<n>_driven.csv (t,x,y,yaw). The driven trajectory is the robot's own pose
estimate (TF), not ground truth: it shows how well the controller tracked the plan, not the
localisation error. Stop with Ctrl+C; plot with plot_execution.py.
"""

import argparse
import csv
import math
import os
import sys

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Recorder(Node):
    def __init__(self, args):
        super().__init__("execution_recorder")
        self.args = args
        os.makedirs(args.out, exist_ok=True)
        self.n = self.next_run_number()
        self.driven = None          # csv writer of the current run
        self.driven_file = None
        self.samples = 0
        self.last_stamp = None
        self.executing = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Path, "/planned_path", self.on_path, 10)
        self.create_subscription(GoalStatusArray, "/follow_path/_action/status", self.on_status,
                                 qos_profile_action_status_default)
        self.create_timer(1.0 / args.rate, self.on_timer)
        self.get_logger().info(f"waiting for /planned_path, writing to {args.out}/{args.label}_<n>_*.csv")

    def next_run_number(self):
        # continue numbering, so restarting the recorder never overwrites earlier runs
        prefix = self.args.label + "_"
        nums = [int(f[len(prefix):].split("_")[0]) for f in os.listdir(self.args.out)
                if f.startswith(prefix) and f.endswith("_plan.csv") and f[len(prefix):].split("_")[0].isdigit()]
        return max(nums, default=0)

    def path_name(self, kind):
        return os.path.join(self.args.out, f"{self.args.label}_{self.n}_{kind}.csv")

    def close_run(self):
        if self.driven_file:
            self.driven_file.close()
            self.get_logger().info(f"run {self.n}: {self.samples} poses in {self.path_name('driven')}")
        self.driven = self.driven_file = None

    def on_path(self, msg):
        if msg.header.frame_id not in ("", self.args.frame):
            self.get_logger().warn(f"path in frame '{msg.header.frame_id}', expected '{self.args.frame}' - ignored")
            return
        self.close_run()
        self.n += 1
        with open(self.path_name("plan"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y"])
            for p in msg.poses:
                w.writerow([f"{p.pose.position.x:.4f}", f"{p.pose.position.y:.4f}"])
        self.driven_file = open(self.path_name("driven"), "w", newline="")
        self.driven = csv.writer(self.driven_file)
        self.driven.writerow(["t", "x", "y", "yaw"])
        self.samples = 0
        self.last_stamp = None
        self.get_logger().info(f"run {self.n}: plan with {len(msg.poses)} poses saved, waiting for FollowPath")

    def on_status(self, msg):
        executing = any(s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list)
        if executing != self.executing:
            self.get_logger().info(f"run {self.n}: FollowPath {'started' if executing else 'finished'}")
            if not executing:
                self.close_run()
        self.executing = executing

    def on_timer(self):
        if not (self.executing and self.driven):
            return
        try:
            t = self.tf.lookup_transform(self.args.frame, self.args.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"no TF {self.args.frame} -> {self.args.base_frame}: {e}",
                                   throttle_duration_sec=2.0)
            return
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tr = t.transform.translation
        self.driven.writerow([f"{stamp:.3f}", f"{tr.x:.4f}", f"{tr.y:.4f}", f"{yaw:.4f}"])
        self.driven_file.flush()
        self.samples += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="file prefix, e.g. cbf or baseline")
    ap.add_argument("--out", default=os.path.join("src", "cbf_rrt_planner", "benchmark", "experiments", "execution"))
    ap.add_argument("--frame", default="map", help="frame of /planned_path")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--rate", type=float, default=20.0, help="TF sampling rate [Hz]")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Recorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_run()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
