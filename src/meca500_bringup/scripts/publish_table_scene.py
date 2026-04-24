#!/usr/bin/env python3

import math
import struct
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_binary_stl(mesh_path: Path, scale: float) -> Mesh:
    data = mesh_path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{mesh_path} is not a valid binary STL")

    triangle_count = struct.unpack("<I", data[80:84])[0]
    inferred_count = (len(data) - 84) // 50
    if triangle_count == 0 or triangle_count != inferred_count:
        triangle_count = inferred_count

    unpack_triangle = struct.Struct("<12fH")
    vertices: list[Point] = []
    triangles: list[MeshTriangle] = []
    vertex_index: dict[tuple[float, float, float], int] = {}

    offset = 84
    for _ in range(triangle_count):
        values = unpack_triangle.unpack_from(data, offset)
        offset += 50

        triangle_vertices = []
        for start in (3, 6, 9):
            key = (
                round(values[start] * scale, 6),
                round(values[start + 1] * scale, 6),
                round(values[start + 2] * scale, 6),
            )
            if key not in vertex_index:
                vertex_index[key] = len(vertices)
                vertices.append(Point(x=key[0], y=key[1], z=key[2]))
            triangle_vertices.append(vertex_index[key])

        if len(set(triangle_vertices)) == 3:
            triangles.append(MeshTriangle(vertex_indices=triangle_vertices))

    mesh = Mesh()
    mesh.vertices = vertices
    mesh.triangles = triangles
    return mesh


class TableScenePublisher(Node):
    def __init__(self) -> None:
        super().__init__("publish_table_scene")

        self.declare_parameter("object_id", "mecatable")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("mesh_path", "")
        self.declare_parameter("mesh_scale", 0.001)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("startup_delay_s", 3.0)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("publish_attempts", 10)

        self.object_id = self.get_parameter("object_id").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        mesh_path = self.get_parameter("mesh_path").get_parameter_value().string_value
        mesh_scale = self.get_parameter("mesh_scale").get_parameter_value().double_value

        if not mesh_path:
            raise RuntimeError("Parameter 'mesh_path' must point to an STL file")

        self.mesh = load_binary_stl(Path(mesh_path), mesh_scale)
        self.pose = Pose()
        self.pose.position.x = self.get_parameter("x").get_parameter_value().double_value
        self.pose.position.y = self.get_parameter("y").get_parameter_value().double_value
        self.pose.position.z = self.get_parameter("z").get_parameter_value().double_value

        roll = self.get_parameter("roll").get_parameter_value().double_value
        pitch = self.get_parameter("pitch").get_parameter_value().double_value
        yaw = self.get_parameter("yaw").get_parameter_value().double_value
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        self.pose.orientation.x = qx
        self.pose.orientation.y = qy
        self.pose.orientation.z = qz
        self.pose.orientation.w = qw

        startup_delay_s = self.get_parameter("startup_delay_s").get_parameter_value().double_value
        publish_period_s = self.get_parameter("publish_period_s").get_parameter_value().double_value
        self.publish_attempts_remaining = (
            self.get_parameter("publish_attempts").get_parameter_value().integer_value
        )

        self.publisher = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.ready_time_ns = (
            self.get_clock().now().nanoseconds + int(startup_delay_s * 1_000_000_000)
        )
        self.timer = self.create_timer(publish_period_s, self.publish_table)

        self.get_logger().info(
            f"Loaded '{mesh_path}' with {len(self.mesh.vertices)} vertices and {len(self.mesh.triangles)} triangles"
        )

    def publish_table(self) -> None:
        if self.get_clock().now().nanoseconds < self.ready_time_ns:
            return

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = self.object_id
        collision_object.operation = CollisionObject.ADD
        collision_object.meshes = [self.mesh]
        collision_object.mesh_poses = [self.pose]

        self.publisher.publish(collision_object)
        self.publish_attempts_remaining -= 1

        self.get_logger().info(
            f"Published table collision object '{self.object_id}' ({self.publish_attempts_remaining} attempts remaining)"
        )

        if self.publish_attempts_remaining <= 0:
            self.timer.cancel()
            self.get_logger().info("Finished publishing table collision object")
            self.destroy_node()
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = TableScenePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
