package com.example.ros_car.viewmodel

import android.app.Application
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Base64
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.example.ros_car.data.*
import com.example.ros_car.rosbridge.RosbridgeClient
import com.example.ros_car.rosbridge.Twist
import com.example.ros_car.rosbridge.Vector3
import com.google.gson.JsonObject
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch

class RosViewModel(application: Application) : AndroidViewModel(application) {
    
    private val rosbridgeClient = RosbridgeClient()
    private val settingsRepository = SettingsRepository(application)
    private val tfManager = TfManager()
    private val mapManager = MapManager()
    
    val connectionState = rosbridgeClient.connectionState
    val errorMessage = rosbridgeClient.errorMessage
    val robotPose = tfManager.robotPose
    val mapBitmap = mapManager.mapBitmap
    val mapMetadata = mapManager.metadata
    
    // 设置
    val lastIp = settingsRepository.lastIp.stateIn(viewModelScope, SharingStarted.Eagerly, "192.168.1.100")
    val lastPort = settingsRepository.lastPort.stateIn(viewModelScope, SharingStarted.Eagerly, "9090")
    val cmdVelTopic = settingsRepository.cmdVelTopic.stateIn(viewModelScope, SharingStarted.Eagerly, "/cmd_vel")
    val mapTopic = settingsRepository.mapTopic.stateIn(viewModelScope, SharingStarted.Eagerly, "/map")
    val tfTopic = settingsRepository.tfTopic.stateIn(viewModelScope, SharingStarted.Eagerly, "/tf")
    val tfStaticTopic = settingsRepository.tfStaticTopic.stateIn(viewModelScope, SharingStarted.Eagerly, "/tf_static")
    val maxLinearSpeed = settingsRepository.maxLinearSpeed.stateIn(viewModelScope, SharingStarted.Eagerly, 0.3f)
    val maxAngularSpeed = settingsRepository.maxAngularSpeed.stateIn(viewModelScope, SharingStarted.Eagerly, 0.8f)
    val mapThrottle = settingsRepository.mapThrottle.stateIn(viewModelScope, SharingStarted.Eagerly, 1000)
    val tfThrottle = settingsRepository.tfThrottle.stateIn(viewModelScope, SharingStarted.Eagerly, 50)
    val cameraTopic = settingsRepository.cameraTopic.stateIn(viewModelScope, SharingStarted.Eagerly, "/camera/image_raw")
    val cameraEnabled = settingsRepository.cameraEnabled.stateIn(viewModelScope, SharingStarted.Eagerly, false)
    
    private val _cameraBitmap = MutableStateFlow<Bitmap?>(null)
    val cameraBitmap: StateFlow<Bitmap?> = _cameraBitmap.asStateFlow()
    
    private var cmdVelPublishJob: Job? = null
    private var currentLinearVel = 0.0
    private var currentAngularVel = 0.0
    private var lastLinearX = 0.0
    private var lastLinearY = 0.0
    private var lastAngularZ = 0.0
    
    fun connect(ip: String, port: String) {
        viewModelScope.launch {
            rosbridgeClient.connect(ip, port)
            settingsRepository.saveLastConnection(ip, port)
        }
    }
    
    fun disconnect() {
        stopCmdVelPublishing()
        rosbridgeClient.disconnect()
        tfManager.clear()
        mapManager.clear()
    }
    
    fun subscribeTopics() {
        viewModelScope.launch {
            // 订阅地图
            rosbridgeClient.subscribe(
                topic = mapTopic.value,
                messageType = "nav_msgs/OccupancyGrid",
                throttleRate = mapThrottle.value,
                queueLength = 1
            ) { msg ->
                mapManager.processMapMessage(msg)
            }
            
            // 订阅TF
            rosbridgeClient.subscribe(
                topic = tfTopic.value,
                messageType = "tf2_msgs/TFMessage",
                throttleRate = tfThrottle.value,
                queueLength = 10
            ) { msg ->
                tfManager.processTfMessage(msg)
            }
            
            // 订阅TF Static
            rosbridgeClient.subscribe(
                topic = tfStaticTopic.value,
                messageType = "tf2_msgs/TFMessage",
                throttleRate = 0,
                queueLength = 10
            ) { msg ->
                tfManager.processTfStaticMessage(msg)
            }
            
            // 订阅相机图像 (如果启用)
            if (cameraEnabled.value) {
                subscribeCameraImage()
            }
        }
    }
    
    fun subscribeCameraImage() {
        viewModelScope.launch {
            rosbridgeClient.subscribe(
                topic = cameraTopic.value,
                messageType = "sensor_msgs/CompressedImage",
                throttleRate = 200, // 5fps
                queueLength = 1
            ) { msg ->
                processCameraImage(msg)
            }
        }
    }
    
    fun unsubscribeCameraImage() {
        viewModelScope.launch {
            rosbridgeClient.unsubscribe(cameraTopic.value)
            _cameraBitmap.value = null
        }
    }
    
    private fun processCameraImage(message: JsonObject) {
        try {
            val data = message.get("data")?.asString ?: return
            
            if (data.isNotEmpty()) {
                // 解码Base64图像数据
                val imageBytes = Base64.decode(data, Base64.DEFAULT)
                val bitmap = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size)
                _cameraBitmap.value = bitmap
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }
    
    fun toggleCamera(enabled: Boolean) {
        viewModelScope.launch {
            settingsRepository.saveCameraEnabled(enabled)
            if (enabled) {
                subscribeCameraImage()
            } else {
                unsubscribeCameraImage()
            }
        }
    }
    
    fun setVelocity(linear: Double, angular: Double) {
        currentLinearVel = linear
        currentAngularVel = angular
        
        // 如果速度不为0且未在发布，启动发布
        if ((linear != 0.0 || angular != 0.0) && cmdVelPublishJob == null) {
            startCmdVelPublishing()
        }
        // 如果速度为0，停止发布但先发一次0速度
        else if (linear == 0.0 && angular == 0.0) {
            publishCmdVel(0.0, 0.0, 0.0)
            stopCmdVelPublishing()
        }
    }
    
    fun setVelocityXYZ(linearX: Double, linearY: Double, angularZ: Double) {
        currentLinearVel = linearX
        currentAngularVel = angularZ
        lastLinearX = linearX
        lastLinearY = linearY
        lastAngularZ = angularZ
        
        // 立即发布当前速度
        publishCmdVel(linearX, linearY, angularZ)
        
        // 如果速度不为0且未在发布，启动定时发布
        if ((linearX != 0.0 || linearY != 0.0 || angularZ != 0.0) && cmdVelPublishJob == null) {
            startCmdVelPublishingXYZ()
        }
        // 如果速度为0，停止发布定时器
        else if (linearX == 0.0 && linearY == 0.0 && angularZ == 0.0) {
            stopCmdVelPublishing()
        }
    }
    
    private fun startCmdVelPublishingXYZ() {
        cmdVelPublishJob = viewModelScope.launch {
            while (true) {
                delay(100) // 10Hz
                publishCmdVel(lastLinearX, lastLinearY, lastAngularZ)
            }
        }
    }
    
    private fun startCmdVelPublishing() {
        cmdVelPublishJob = viewModelScope.launch {
            while (true) {
                publishCmdVel(currentLinearVel, 0.0, currentAngularVel)
                delay(100) // 10Hz
            }
        }
    }
    
    private fun stopCmdVelPublishing() {
        cmdVelPublishJob?.cancel()
        cmdVelPublishJob = null
    }
    
    private fun publishCmdVel(linearX: Double, linearY: Double, angularZ: Double) {
        val twist = Twist(
            linear = Vector3(x = linearX, y = linearY, z = 0.0),
            angular = Vector3(x = 0.0, y = 0.0, z = angularZ)
        )
        rosbridgeClient.publish(
            topic = cmdVelTopic.value,
            messageType = "geometry_msgs/Twist",
            message = twist
        )
    }
    
    // 设置保存方法
    fun saveMaxLinearSpeed(speed: Float) {
        viewModelScope.launch {
            settingsRepository.saveMaxLinearSpeed(speed)
        }
    }
    
    fun saveMaxAngularSpeed(speed: Float) {
        viewModelScope.launch {
            settingsRepository.saveMaxAngularSpeed(speed)
        }
    }
    
    fun saveCmdVelTopic(topic: String) {
        viewModelScope.launch {
            settingsRepository.saveCmdVelTopic(topic)
        }
    }
    
    fun saveMapTopic(topic: String) {
        viewModelScope.launch {
            settingsRepository.saveMapTopic(topic)
        }
    }
    
    fun saveTfTopic(topic: String) {
        viewModelScope.launch {
            settingsRepository.saveTfTopic(topic)
        }
    }
    
    fun saveMapThrottle(throttle: Int) {
        viewModelScope.launch {
            settingsRepository.saveMapThrottle(throttle)
        }
    }
    
    fun saveTfThrottle(throttle: Int) {
        viewModelScope.launch {
            settingsRepository.saveTfThrottle(throttle)
        }
    }
    
    fun saveCameraTopic(topic: String) {
        viewModelScope.launch {
            settingsRepository.saveCameraTopic(topic)
        }
    }
    
    override fun onCleared() {
        super.onCleared()
        stopCmdVelPublishing()
        publishCmdVel(0.0, 0.0, 0.0)
        disconnect()
    }
}
