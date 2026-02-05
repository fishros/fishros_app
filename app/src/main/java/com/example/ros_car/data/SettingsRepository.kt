package com.example.ros_car.data

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.*
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "ros_settings")

class SettingsRepository(private val context: Context) {
    
    companion object {
        val LAST_IP = stringPreferencesKey("last_ip")
        val LAST_PORT = stringPreferencesKey("last_port")
        val CMD_VEL_TOPIC = stringPreferencesKey("cmd_vel_topic")
        val MAP_TOPIC = stringPreferencesKey("map_topic")
        val TF_TOPIC = stringPreferencesKey("tf_topic")
        val TF_STATIC_TOPIC = stringPreferencesKey("tf_static_topic")
        val CAMERA_TOPIC = stringPreferencesKey("camera_topic")
        val CAMERA_ENABLED = booleanPreferencesKey("camera_enabled")
        val SCAN_TOPIC = stringPreferencesKey("scan_topic")
        val SCAN_ENABLED = booleanPreferencesKey("scan_enabled")
        val MAX_LINEAR_SPEED = floatPreferencesKey("max_linear_speed")
        val MAX_ANGULAR_SPEED = floatPreferencesKey("max_angular_speed")
        val BASE_FRAME = stringPreferencesKey("base_frame")
        val FIXED_FRAME = stringPreferencesKey("fixed_frame")
        val MAP_THROTTLE = intPreferencesKey("map_throttle")
        val TF_THROTTLE = intPreferencesKey("tf_throttle")
    }

    val lastIp: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[LAST_IP] ?: "192.168.1.100"
    }

    val lastPort: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[LAST_PORT] ?: "9090"
    }

    val cmdVelTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[CMD_VEL_TOPIC] ?: "/cmd_vel"
    }

    val mapTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[MAP_TOPIC] ?: "/map"
    }

    val tfTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[TF_TOPIC] ?: "/tf"
    }

    val tfStaticTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[TF_STATIC_TOPIC] ?: "/tf_static"
    }

    val maxLinearSpeed: Flow<Float> = context.dataStore.data.map { preferences ->
        preferences[MAX_LINEAR_SPEED] ?: 0.3f
    }

    val maxAngularSpeed: Flow<Float> = context.dataStore.data.map { preferences ->
        preferences[MAX_ANGULAR_SPEED] ?: 0.8f
    }

    val baseFrame: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[BASE_FRAME] ?: "base_link"
    }

    val fixedFrame: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[FIXED_FRAME] ?: "map"
    }

    val mapThrottle: Flow<Int> = context.dataStore.data.map { preferences ->
        preferences[MAP_THROTTLE] ?: 1000 // 1秒
    }

    val tfThrottle: Flow<Int> = context.dataStore.data.map { preferences ->
        preferences[TF_THROTTLE] ?: 50 // 50ms
    }

    val cameraTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[CAMERA_TOPIC] ?: "/camera/image_raw"
    }

    val cameraEnabled: Flow<Boolean> = context.dataStore.data.map { preferences ->
        preferences[CAMERA_ENABLED] ?: false
    }

    val scanTopic: Flow<String> = context.dataStore.data.map { preferences ->
        preferences[SCAN_TOPIC] ?: "/scan"
    }

    val scanEnabled: Flow<Boolean> = context.dataStore.data.map { preferences ->
        preferences[SCAN_ENABLED] ?: true
    }

    suspend fun saveLastConnection(ip: String, port: String) {
        context.dataStore.edit { preferences ->
            preferences[LAST_IP] = ip
            preferences[LAST_PORT] = port
        }
    }

    suspend fun saveCmdVelTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[CMD_VEL_TOPIC] = topic
        }
    }

    suspend fun saveMapTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[MAP_TOPIC] = topic
        }
    }

    suspend fun saveTfTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[TF_TOPIC] = topic
        }
    }

    suspend fun saveTfStaticTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[TF_STATIC_TOPIC] = topic
        }
    }

    suspend fun saveMaxLinearSpeed(speed: Float) {
        context.dataStore.edit { preferences ->
            preferences[MAX_LINEAR_SPEED] = speed
        }
    }

    suspend fun saveMaxAngularSpeed(speed: Float) {
        context.dataStore.edit { preferences ->
            preferences[MAX_ANGULAR_SPEED] = speed
        }
    }

    suspend fun saveBaseFrame(frame: String) {
        context.dataStore.edit { preferences ->
            preferences[BASE_FRAME] = frame
        }
    }

    suspend fun saveFixedFrame(frame: String) {
        context.dataStore.edit { preferences ->
            preferences[FIXED_FRAME] = frame
        }
    }

    suspend fun saveMapThrottle(throttle: Int) {
        context.dataStore.edit { preferences ->
            preferences[MAP_THROTTLE] = throttle
        }
    }

    suspend fun saveTfThrottle(throttle: Int) {
        context.dataStore.edit { preferences ->
            preferences[TF_THROTTLE] = throttle
        }
    }

    suspend fun saveCameraTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[CAMERA_TOPIC] = topic
        }
    }

    suspend fun saveCameraEnabled(enabled: Boolean) {
        context.dataStore.edit { preferences ->
            preferences[CAMERA_ENABLED] = enabled
        }
    }

    suspend fun saveScanTopic(topic: String) {
        context.dataStore.edit { preferences ->
            preferences[SCAN_TOPIC] = topic
        }
    }

    suspend fun saveScanEnabled(enabled: Boolean) {
        context.dataStore.edit { preferences ->
            preferences[SCAN_ENABLED] = enabled
        }
    }
}
