package com.example.ros_car.rosbridge

import com.google.gson.Gson
import com.google.gson.JsonObject
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import okhttp3.*
import java.util.concurrent.TimeUnit
import android.util.Log

class RosbridgeClient {
    private val TAG = "RosbridgeClient"
    private var webSocket: WebSocket? = null
    private val client = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.SECONDS)
        .writeTimeout(5, TimeUnit.SECONDS)
        .build()

    private val gson = Gson()
    private val subscriptions = mutableMapOf<String, (JsonObject) -> Unit>()
    
    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState> = _connectionState
    
    private val _errorMessage = MutableStateFlow<String?>(null)
    val errorMessage: StateFlow<String?> = _errorMessage

    enum class ConnectionState {
        DISCONNECTED, CONNECTING, CONNECTED, FAILED
    }

    fun connect(ip: String, port: String) {
        if (_connectionState.value == ConnectionState.CONNECTING ||
            _connectionState.value == ConnectionState.CONNECTED) {
            return
        }

        _connectionState.value = ConnectionState.CONNECTING
        _errorMessage.value = null

        val url = "ws://$ip:$port"
        val request = Request.Builder()
            .url(url)
            .build()

        val listener = object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                Log.d(TAG, "WebSocket connected")
                _connectionState.value = ConnectionState.CONNECTED
                _errorMessage.value = null
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                try {
                    val json = gson.fromJson(text, JsonObject::class.java)
                    
                    // 处理订阅消息
                    if (json.has("topic")) {
                        val topic = json.get("topic").asString
                        subscriptions[topic]?.let { callback ->
                            json.getAsJsonObject("msg")?.let { msg ->
                                callback(msg)
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e(TAG, "Error parsing message", e)
                }
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                Log.e(TAG, "WebSocket failure", t)
                _connectionState.value = ConnectionState.FAILED
                _errorMessage.value = t.message ?: "连接失败"
                this@RosbridgeClient.webSocket = null
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                Log.d(TAG, "WebSocket closed: $reason")
                _connectionState.value = ConnectionState.DISCONNECTED
                this@RosbridgeClient.webSocket = null
            }
        }

        webSocket = client.newWebSocket(request, listener)
    }

    fun disconnect() {
        webSocket?.close(1000, "User disconnect")
        webSocket = null
        subscriptions.clear()
        _connectionState.value = ConnectionState.DISCONNECTED
    }

    fun publish(topic: String, messageType: String, message: Any) {
        if (_connectionState.value != ConnectionState.CONNECTED) {
            Log.w(TAG, "Cannot publish: not connected")
            return
        }

        val rosMessage = mapOf(
            "op" to "publish",
            "topic" to topic,
            "type" to messageType,
            "msg" to message
        )

        try {
            val json = gson.toJson(rosMessage)
            webSocket?.send(json)
        } catch (e: Exception) {
            Log.e(TAG, "Error publishing message", e)
        }
    }

    fun subscribe(
        topic: String,
        messageType: String,
        throttleRate: Int = 0,
        queueLength: Int = 1,
        callback: (JsonObject) -> Unit
    ): String {
        if (_connectionState.value != ConnectionState.CONNECTED) {
            Log.w(TAG, "Cannot subscribe: not connected")
            return ""
        }

        subscriptions[topic] = callback

        val subscribeMessage = mutableMapOf(
            "op" to "subscribe",
            "topic" to topic,
            "type" to messageType,
            "queue_length" to queueLength
        )

        if (throttleRate > 0) {
            subscribeMessage["throttle_rate"] = throttleRate
        }

        try {
            val json = gson.toJson(subscribeMessage)
            webSocket?.send(json)
            Log.d(TAG, "Subscribed to $topic")
        } catch (e: Exception) {
            Log.e(TAG, "Error subscribing to topic", e)
        }

        return topic
    }

    fun unsubscribe(topic: String) {
        if (_connectionState.value != ConnectionState.CONNECTED) {
            return
        }

        subscriptions.remove(topic)

        val unsubscribeMessage = mapOf(
            "op" to "unsubscribe",
            "topic" to topic
        )

        try {
            val json = gson.toJson(unsubscribeMessage)
            webSocket?.send(json)
            Log.d(TAG, "Unsubscribed from $topic")
        } catch (e: Exception) {
            Log.e(TAG, "Error unsubscribing from topic", e)
        }
    }

    fun isConnected(): Boolean {
        return _connectionState.value == ConnectionState.CONNECTED
    }
}
