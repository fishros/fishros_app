package com.example.ros_car

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.example.ros_car.ui.screens.*
import com.example.ros_car.ui.theme.Ros_carTheme
import com.example.ros_car.viewmodel.RosViewModel

class MainActivity : ComponentActivity() {
    private val viewModel: RosViewModel by viewModels()
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        setContent {
            Ros_carTheme {
                RosCarApp(viewModel)
            }
        }
    }
}

@Composable
fun RosCarApp(viewModel: RosViewModel) {
    var currentScreen by remember { mutableStateOf<Screen>(Screen.Connection) }
    val lifecycleOwner = LocalLifecycleOwner.current
    
    // 监听生命周期，暂停时发送0速度
    DisposableEffect(lifecycleOwner) {
        val observer = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_PAUSE -> {
                    // 暂停时停止机器人
                    viewModel.setVelocity(0.0, 0.0)
                }
                Lifecycle.Event.ON_DESTROY -> {
                    // 销毁时断开连接
                    viewModel.disconnect()
                }
                else -> {}
            }
        }
        
        lifecycleOwner.lifecycle.addObserver(observer)
        
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
        }
    }
    
    when (currentScreen) {
        Screen.Connection -> {
            ConnectionScreen(
                viewModel = viewModel,
                onConnectionSuccess = {
                    currentScreen = Screen.Home
                }
            )
        }
        Screen.Home -> {
            HomeScreen(
                onNavigateToControl = {
                    currentScreen = Screen.Control
                },
                onNavigateToSettings = {
                    currentScreen = Screen.Settings
                }
            )
        }
        Screen.Control -> {
            ControlScreen(
                viewModel = viewModel,
                onBack = {
                    // 返回前停止机器人
                    viewModel.setVelocity(0.0, 0.0)
                    currentScreen = Screen.Home
                }
            )
        }
        Screen.Settings -> {
            SettingsScreen(
                viewModel = viewModel,
                onBack = {
                    currentScreen = Screen.Home
                },
                onDisconnect = {
                    viewModel.disconnect()
                    currentScreen = Screen.Connection
                }
            )
        }
    }
}

sealed class Screen {
    object Connection : Screen()
    object Home : Screen()
    object Control : Screen()
    object Settings : Screen()
}
