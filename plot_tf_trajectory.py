#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from tf2_ros import Buffer, TransformException


def open_bag(bag_path: str, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    storage_options = StorageOptions(
        uri=bag_path,
        storage_id=storage_id,
    )
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def main():
    parser = argparse.ArgumentParser(
        description="Plot map -> base_footprint XY trajectory from /tf and /tf_static in a ROS 2 bag."
    )
    parser.add_argument(
        "bag",
        help="Path to rosbag directory or .mcap file, e.g. tf_recording or tf_recording_0.mcap",
    )
    parser.add_argument(
        "--target-frame",
        default="map",
        help="Target/global frame. Default: map",
    )
    parser.add_argument(
        "--source-frame",
        default="base_footprint",
        help="Robot/source frame. Default: base_footprint",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="rosbag2 storage id. Default: mcap",
    )
    parser.add_argument(
        "--output",
        default="trajectory.png",
        help="Output plot filename. Default: trajectory.png",
    )
    args = parser.parse_args()

    bag_path = str(Path(args.bag).expanduser())

    rclpy.init()

    reader = open_bag(bag_path, args.storage_id)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    if "/tf" not in topic_types and "/tf_static" not in topic_types:
        raise RuntimeError("Bag does not contain /tf or /tf_static.")

    tf_buffer = Buffer()

    times = []
    xs = []
    ys = []
    yaws = []

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()

        if topic not in ["/tf", "/tf_static"]:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        for transform in msg.transforms:
            if topic == "/tf_static":
                tf_buffer.set_transform_static(transform, "bag")
            else:
                tf_buffer.set_transform(transform, "bag")

        try:
            transform = tf_buffer.lookup_transform(
                args.target_frame,
                args.source_frame,
                Time(),
            )
        except TransformException:
            continue

        t = bag_time_ns * 1e-9
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        times.append(t)
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)

    rclpy.shutdown()

    if not xs:
        raise RuntimeError(
            f"No valid {args.target_frame} -> {args.source_frame} transforms found."
        )

    t0 = times[0]
    times = [t - t0 for t in times]

    plt.figure()
    plt.plot(xs, ys, linewidth=2)
    plt.scatter(xs[0], ys[0], marker="o", label="start")
    plt.scatter(xs[-1], ys[-1], marker="x", label="end")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel(f"x in {args.target_frame} [m]")
    plt.ylabel(f"y in {args.target_frame} [m]")
    plt.title(f"{args.target_frame} -> {args.source_frame} trajectory")
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.output, dpi=200)
    plt.show()

    print(f"Saved plot to: {args.output}")
    print(f"Samples plotted: {len(xs)}")
    print(f"Start: x={xs[0]:.3f}, y={ys[0]:.3f}")
    print(f"End:   x={xs[-1]:.3f}, y={ys[-1]:.3f}")


def quaternion_to_yaw(x, y, z, w):
    import math

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


if __name__ == "__main__":
    main()