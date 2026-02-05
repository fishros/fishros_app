package com.example.ros_car.data

import android.util.Log
import com.example.ros_car.rosbridge.LaserScan
import com.example.ros_car.rosbridge.LaserScanPoint
import com.google.gson.JsonObject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlin.math.cos
import kotlin.math.sin

class LaserScanManager(private val tfManager: TfManager) {
    private val TAG = "LaserScanManager"
    
    private val _laserScan = MutableStateFlow<LaserScan?>(null)
    val laserScan: StateFlow<LaserScan?> = _laserScan
    
    fun processLaserScanMessage(msg: JsonObject) {
        try {
            // 安全获取字段，处理 null 值
            val angleMinElement = msg.get("angle_min")
            val angleMaxElement = msg.get("angle_max")
            val angleIncrementElement = msg.get("angle_increment")
            val rangeMinElement = msg.get("range_min")
            val rangeMaxElement = msg.get("range_max")
            val rangesArray = msg.getAsJsonArray("ranges")
            
            // 检查必需字段是否存在且非 null
            if (angleMinElement == null || angleMinElement.isJsonNull ||
                angleMaxElement == null || angleMaxElement.isJsonNull ||
                angleIncrementElement == null || angleIncrementElement.isJsonNull ||
                rangeMinElement == null || rangeMinElement.isJsonNull ||
                rangeMaxElement == null || rangeMaxElement.isJsonNull ||
                rangesArray == null) {
                Log.w(TAG, "LaserScan message has null fields")
                return
            }
            
            val angleMin = angleMinElement.asFloat
            val angleMax = angleMaxElement.asFloat
            val angleIncrement = angleIncrementElement.asFloat
            val rangeMin = rangeMinElement.asFloat
            val rangeMax = rangeMaxElement.asFloat
            
            val ranges = FloatArray(rangesArray.size()) { i ->
                val element = rangesArray.get(i)
                if (element.isJsonNull) {
                    0f
                } else {
                    val value = element.asFloat
                    // 过滤无效值 (inf, nan)
                    if (value.isInfinite() || value.isNaN() || value < rangeMin || value > rangeMax) {
                        0f
                    } else {
                        value
                    }
                }
            }
            
            // 获取当前激光雷达位姿（map→laser_link），立即转换为世界坐标
            val laserPose = tfManager.laserPose.value
            
            // 调试信息
            Log.d(TAG, "LaserPose: x=${laserPose.x}, y=${laserPose.y}, theta=${laserPose.theta}")
            
            val worldPoints = mutableListOf<LaserScanPoint>()
            
            var angle = angleMin
            
            for (i in ranges.indices) {
                val range = ranges[i]
                if (range > rangeMin && range < rangeMax) {
                    // 激光点在激光雷达坐标系下的位置
                    val pointX_laser = range * cos(angle)
                    val pointY_laser = range * sin(angle)
                    
                    // 因为laserPose已经是map→laser_link的完整变换
                    // 所以需要应用这个变换将激光坐标系的点转到世界坐标系
                    val cos_theta = cos(laserPose.theta.toFloat())
                    val sin_theta = sin(laserPose.theta.toFloat())
                    
                    val pointX_world = pointX_laser * cos_theta - pointY_laser * sin_theta
                    val pointY_world = pointX_laser * sin_theta + pointY_laser * cos_theta
                    
                    val worldX = laserPose.x.toFloat() + pointX_world
                    val worldY = laserPose.y.toFloat() + pointY_world
                    
                    worldPoints.add(LaserScanPoint(worldX, worldY))
                }
                angle += angleIncrement
            }
            
            val scan = LaserScan(
                angleMin = angleMin,
                angleMax = angleMax,
                angleIncrement = angleIncrement,
                rangeMin = rangeMin,
                rangeMax = rangeMax,
                ranges = ranges,
                worldPoints = worldPoints
            )
            
            _laserScan.value = scan
            Log.d(TAG, "LaserScan processed: ${ranges.size} points, ${worldPoints.size} valid points, angle_min=$angleMin, angle_max=$angleMax")
            
        } catch (e: Exception) {
            Log.e(TAG, "Error processing laser scan message", e)
        }
    }
    
    fun clear() {
        _laserScan.value = null
    }
}
