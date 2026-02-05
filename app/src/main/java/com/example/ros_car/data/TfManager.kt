package com.example.ros_car.data

import android.util.Log
import com.example.ros_car.rosbridge.Quaternion
import com.example.ros_car.rosbridge.RobotPose
import com.example.ros_car.rosbridge.Transform
import com.example.ros_car.rosbridge.Vector3
import com.google.gson.JsonObject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import java.util.ArrayDeque

data class TFTransform(
    val parentFrame: String,
    val childFrame: String,
    val transform: Transform,
    val timestamp: Long = System.currentTimeMillis()
)

class TfManager {
    private val TAG = "TfManager"
    private val transforms = mutableMapOf<String, TFTransform>()
    private val staticTransforms = mutableMapOf<String, TFTransform>()
    
    private val _robotPose = MutableStateFlow(RobotPose())
    val robotPose: StateFlow<RobotPose> = _robotPose
    
    private val _laserPose = MutableStateFlow(RobotPose())
    val laserPose: StateFlow<RobotPose> = _laserPose

    private data class Pose2D(
        val x: Double,
        val y: Double,
        val yaw: Double
    )

    private data class Edge(
        val to: String,
        val pose: Pose2D
    )

    fun processTfMessage(msg: JsonObject) {
        try {
            val transformsArray = msg.getAsJsonArray("transforms")
            
            for (element in transformsArray) {
                val transformObj = element.asJsonObject
                val header = transformObj.getAsJsonObject("header")
                val parentFrame = header.get("frame_id").asString.removePrefix("/")
                val childFrame = transformObj.get("child_frame_id").asString.removePrefix("/")
                
                val transformData = transformObj.getAsJsonObject("transform")
                val translation = transformData.getAsJsonObject("translation")
                val rotation = transformData.getAsJsonObject("rotation")
                
                val transform = Transform(
                    translation = Vector3(
                        x = translation.get("x").asDouble,
                        y = translation.get("y").asDouble,
                        z = translation.get("z").asDouble
                    ),
                    rotation = Quaternion(
                        x = rotation.get("x").asDouble,
                        y = rotation.get("y").asDouble,
                        z = rotation.get("z").asDouble,
                        w = rotation.get("w").asDouble
                    )
                )
                
                val tfTransform = TFTransform(parentFrame, childFrame, transform)
                transforms["${parentFrame}_to_${childFrame}"] = tfTransform
            }

            updatePoses()
        } catch (e: Exception) {
            Log.e(TAG, "Error processing TF message", e)
        }
    }

    fun processTfStaticMessage(msg: JsonObject) {
        try {
            val transformsArray = msg.getAsJsonArray("transforms")
            
            for (element in transformsArray) {
                val transformObj = element.asJsonObject
                val header = transformObj.getAsJsonObject("header")
                val parentFrame = header.get("frame_id").asString.removePrefix("/")
                val childFrame = transformObj.get("child_frame_id").asString.removePrefix("/")
                
                val transformData = transformObj.getAsJsonObject("transform")
                val translation = transformData.getAsJsonObject("translation")
                val rotation = transformData.getAsJsonObject("rotation")
                
                val transform = Transform(
                    translation = Vector3(
                        x = translation.get("x").asDouble,
                        y = translation.get("y").asDouble,
                        z = translation.get("z").asDouble
                    ),
                    rotation = Quaternion(
                        x = rotation.get("x").asDouble,
                        y = rotation.get("y").asDouble,
                        z = rotation.get("z").asDouble,
                        w = rotation.get("w").asDouble
                    )
                )
                
                val tfTransform = TFTransform(parentFrame, childFrame, transform)
                staticTransforms["${parentFrame}_to_${childFrame}"] = tfTransform
            }

            updatePoses()
        } catch (e: Exception) {
            Log.e(TAG, "Error processing TF static message", e)
        }
    }

    fun getTransform(parentFrame: String, childFrame: String): TFTransform? {
        val key = "${parentFrame}_to_${childFrame}"
        return transforms[key] ?: staticTransforms[key]
    }

    fun resolvePose(parentFrame: String, childFrame: String): RobotPose? {
        val pose = resolvePose2D(parentFrame, childFrame) ?: return null
        return RobotPose(pose.x, pose.y, pose.yaw)
    }

    private fun resolvePose2D(parentFrame: String, childFrame: String): Pose2D? {
        if (parentFrame == childFrame) {
            return Pose2D(0.0, 0.0, 0.0)
        }

        val adjacency = buildAdjacency()
        val visited = mutableSetOf<String>()
        val queue: ArrayDeque<Pair<String, Pose2D>> = ArrayDeque()
        queue.add(parentFrame to Pose2D(0.0, 0.0, 0.0))
        visited.add(parentFrame)

        while (queue.isNotEmpty()) {
            val (current, currentPose) = queue.removeFirst()
            if (current == childFrame) {
                return currentPose
            }
            for (edge in adjacency[current].orEmpty()) {
                if (edge.to in visited) continue
                val nextPose = compose(currentPose, edge.pose)
                queue.add(edge.to to nextPose)
                visited.add(edge.to)
            }
        }

        return null
    }

    private fun buildAdjacency(): Map<String, List<Edge>> {
        val adjacency = mutableMapOf<String, MutableList<Edge>>()
        val allTransforms = transforms.values + staticTransforms.values

        for (tf in allTransforms) {
            val pose = transformToPose2D(tf.transform)
            adjacency.getOrPut(tf.parentFrame) { mutableListOf() }
                .add(Edge(tf.childFrame, pose))
            adjacency.getOrPut(tf.childFrame) { mutableListOf() }
                .add(Edge(tf.parentFrame, invert(pose)))
        }

        return adjacency
    }

    private fun transformToPose2D(transform: Transform): Pose2D {
        return Pose2D(
            x = transform.translation.x,
            y = transform.translation.y,
            yaw = transform.rotation.toYaw()
        )
    }

    private fun compose(a: Pose2D, b: Pose2D): Pose2D {
        val cosA = kotlin.math.cos(a.yaw)
        val sinA = kotlin.math.sin(a.yaw)
        val x = a.x + b.x * cosA - b.y * sinA
        val y = a.y + b.x * sinA + b.y * cosA
        val yaw = a.yaw + b.yaw
        return Pose2D(x, y, yaw)
    }

    private fun invert(pose: Pose2D): Pose2D {
        val cosA = kotlin.math.cos(pose.yaw)
        val sinA = kotlin.math.sin(pose.yaw)
        val x = -(cosA * pose.x + sinA * pose.y)
        val y = sinA * pose.x - cosA * pose.y
        val yaw = -pose.yaw
        return Pose2D(x, y, yaw)
    }

    private fun updatePoses() {
        val mapRobot = resolvePose2D("map", "base_link")
            ?: resolvePose2D("map", "base_footprint")
        val odomRobot = resolvePose2D("odom", "base_link")
            ?: resolvePose2D("odom", "base_footprint")

        when {
            mapRobot != null -> _robotPose.value = RobotPose(mapRobot.x, mapRobot.y, mapRobot.yaw)
            odomRobot != null -> _robotPose.value = RobotPose(odomRobot.x, odomRobot.y, odomRobot.yaw)
        }

        val mapLaser = resolvePose2D("map", "laser_link")
        val odomLaser = resolvePose2D("odom", "laser_link")

        when {
            mapLaser != null -> _laserPose.value = RobotPose(mapLaser.x, mapLaser.y, mapLaser.yaw)
            odomLaser != null -> _laserPose.value = RobotPose(odomLaser.x, odomLaser.y, odomLaser.yaw)
        }
    }

    fun clear() {
        transforms.clear()
        staticTransforms.clear()
        _robotPose.value = RobotPose()
        _laserPose.value = RobotPose()
    }
}
