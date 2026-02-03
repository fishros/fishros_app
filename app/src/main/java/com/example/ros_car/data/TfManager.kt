package com.example.ros_car.data

import android.util.Log
import com.example.ros_car.rosbridge.Quaternion
import com.example.ros_car.rosbridge.RobotPose
import com.example.ros_car.rosbridge.Transform
import com.example.ros_car.rosbridge.Vector3
import com.google.gson.JsonObject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

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
                
                // 更新机器人位姿（假设从map或odom到base_link）
                if ((parentFrame == "map" || parentFrame == "odom") && 
                    (childFrame == "base_link" || childFrame == "base_footprint")) {
                    _robotPose.value = RobotPose(
                        x = transform.translation.x,
                        y = transform.translation.y,
                        theta = transform.rotation.toYaw()
                    )
                }
            }
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
        } catch (e: Exception) {
            Log.e(TAG, "Error processing TF static message", e)
        }
    }

    fun getTransform(parentFrame: String, childFrame: String): TFTransform? {
        val key = "${parentFrame}_to_${childFrame}"
        return transforms[key] ?: staticTransforms[key]
    }

    fun clear() {
        transforms.clear()
        _robotPose.value = RobotPose()
    }
}
