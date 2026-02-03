package com.example.ros_car.rosbridge

data class Twist(
    val linear: Vector3 = Vector3(),
    val angular: Vector3 = Vector3()
)

data class Vector3(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0
)

data class Quaternion(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0,
    val w: Double = 1.0
) {
    fun toYaw(): Double {
        val siny_cosp = 2.0 * (w * z + x * y)
        val cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return Math.atan2(siny_cosp, cosy_cosp)
    }
}

data class Pose(
    val position: Vector3 = Vector3(),
    val orientation: Quaternion = Quaternion()
)

data class Transform(
    val translation: Vector3 = Vector3(),
    val rotation: Quaternion = Quaternion()
)

data class RobotPose(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val theta: Double = 0.0
)
