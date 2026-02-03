package com.example.ros_car.data

import android.graphics.Bitmap
import android.graphics.Color
import android.util.Log
import com.google.gson.JsonObject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

data class MapMetadata(
    val resolution: Float,
    val width: Int,
    val height: Int,
    val originX: Float,
    val originY: Float,
    val originYaw: Float
)

class MapManager {
    private val TAG = "MapManager"
    
    private val _mapBitmap = MutableStateFlow<Bitmap?>(null)
    val mapBitmap: StateFlow<Bitmap?> = _mapBitmap
    
    private val _metadata = MutableStateFlow<MapMetadata?>(null)
    val metadata: StateFlow<MapMetadata?> = _metadata

    fun processMapMessage(msg: JsonObject) {
        try {
            val info = msg.getAsJsonObject("info")
            val resolution = info.get("resolution").asFloat
            val width = info.get("width").asInt
            val height = info.get("height").asInt
            
            val origin = info.getAsJsonObject("origin")
            val position = origin.getAsJsonObject("position")
            val orientation = origin.getAsJsonObject("orientation")
            
            val originX = position.get("x").asFloat
            val originY = position.get("y").asFloat
            
            // 计算yaw角度
            val qz = orientation.get("z").asDouble
            val qw = orientation.get("w").asDouble
            val siny_cosp = 2.0 * qw * qz
            val cosy_cosp = 1.0 - 2.0 * qz * qz
            val originYaw = Math.atan2(siny_cosp, cosy_cosp).toFloat()
            
            val metadata = MapMetadata(resolution, width, height, originX, originY, originYaw)
            _metadata.value = metadata
            
            // 解析地图数据
            val dataArray = msg.getAsJsonArray("data")
            val data = IntArray(dataArray.size()) { i ->
                dataArray.get(i).asInt
            }
            
            // 创建Bitmap
            val bitmap = createMapBitmap(width, height, data)
            _mapBitmap.value = bitmap
            
            Log.d(TAG, "Map processed: ${width}x${height}, resolution=$resolution")
        } catch (e: Exception) {
            Log.e(TAG, "Error processing map message", e)
        }
    }

    private fun createMapBitmap(width: Int, height: Int, data: IntArray): Bitmap {
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        
        for (y in 0 until height) {
            for (x in 0 until width) {
                val idx = y * width + x
                val value = data[idx]
                
                val color = when {
                    value == -1 -> Color.rgb(128, 128, 128) // 未知：灰色
                    value == 0 -> Color.rgb(255, 255, 255)  // 空闲：白色
                    value == 100 -> Color.rgb(0, 0, 0)       // 障碍：黑色
                    else -> {
                        // 根据占用概率渐变
                        val gray = 255 - (value * 255 / 100)
                        Color.rgb(gray, gray, gray)
                    }
                }
                
                // 注意：栅格地图的y轴需要翻转
                bitmap.setPixel(x, height - 1 - y, color)
            }
        }
        
        return bitmap
    }

    fun worldToMap(worldX: Double, worldY: Double): Pair<Int, Int>? {
        val meta = _metadata.value ?: return null
        
        // 简化版本：假设originYaw=0
        val mapX = ((worldX - meta.originX) / meta.resolution).toInt()
        val mapY = ((worldY - meta.originY) / meta.resolution).toInt()
        
        return if (mapX in 0 until meta.width && mapY in 0 until meta.height) {
            Pair(mapX, mapY)
        } else {
            null
        }
    }

    fun clear() {
        _mapBitmap.value = null
        _metadata.value = null
    }
}
