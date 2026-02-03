package com.example.ros_car.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.ros_car.rosbridge.RosbridgeClient
import com.example.ros_car.viewmodel.RosViewModel

@Composable
fun SettingsScreen(
    viewModel: RosViewModel,
    onBack: () -> Unit,
    onDisconnect: () -> Unit
) {
    val connectionState by viewModel.connectionState.collectAsState()
    val cmdVelTopic by viewModel.cmdVelTopic.collectAsState()
    val mapTopic by viewModel.mapTopic.collectAsState()
    val tfTopic by viewModel.tfTopic.collectAsState()
    val cameraTopic by viewModel.cameraTopic.collectAsState()
    val maxLinearSpeed by viewModel.maxLinearSpeed.collectAsState()
    val maxAngularSpeed by viewModel.maxAngularSpeed.collectAsState()
    val mapThrottle by viewModel.mapThrottle.collectAsState()
    val tfThrottle by viewModel.tfThrottle.collectAsState()
    
    var linearSpeedText by remember { mutableStateOf(maxLinearSpeed.toString()) }
    var angularSpeedText by remember { mutableStateOf(maxAngularSpeed.toString()) }
    var cmdVelTopicText by remember { mutableStateOf(cmdVelTopic) }
    var mapTopicText by remember { mutableStateOf(mapTopic) }
    var tfTopicText by remember { mutableStateOf(tfTopic) }
    var cameraTopicText by remember { mutableStateOf(cameraTopic) }
    var mapThrottleText by remember { mutableStateOf(mapThrottle.toString()) }
    var tfThrottleText by remember { mutableStateOf(tfThrottle.toString()) }
    
    // 同步状态
    LaunchedEffect(maxLinearSpeed) { linearSpeedText = maxLinearSpeed.toString() }
    LaunchedEffect(maxAngularSpeed) { angularSpeedText = maxAngularSpeed.toString() }
    LaunchedEffect(cmdVelTopic) { cmdVelTopicText = cmdVelTopic }
    LaunchedEffect(mapTopic) { mapTopicText = mapTopic }
    LaunchedEffect(tfTopic) { tfTopicText = tfTopic }
    LaunchedEffect(cameraTopic) { cameraTopicText = cameraTopic }
    LaunchedEffect(mapThrottle) { mapThrottleText = mapThrottle.toString() }
    LaunchedEffect(tfThrottle) { tfThrottleText = tfThrottle.toString() }
    
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(
                Brush.verticalGradient(
                    colors = listOf(
                        Color(0xFF1a1a1a),
                        Color(0xFF2d2d2d),
                        Color(0xFF1a1a1a)
                    )
                )
            )
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .verticalScroll(rememberScrollState())
                .padding(32.dp)
        ) {
            // 返回按钮
            IconButton(
                onClick = onBack,
                modifier = Modifier.padding(bottom = 16.dp)
            ) {
                Surface(
                    shape = RoundedCornerShape(12.dp),
                    color = Color(0xFF374151),
                    modifier = Modifier.size(48.dp)
                ) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text("←", fontSize = 24.sp, color = Color.White)
                    }
                }
            }
            
            Text(
                text = "设置",
                fontSize = 36.sp,
                fontWeight = FontWeight.Bold,
                color = Color.White,
                modifier = Modifier.padding(bottom = 32.dp)
            )
            
            // 连接状态
            SettingsSection(title = "连接状态") {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    modifier = Modifier.padding(vertical = 8.dp)
                ) {
                    Box(
                        modifier = Modifier
                            .size(12.dp)
                            .background(
                                color = if (connectionState == RosbridgeClient.ConnectionState.CONNECTED)
                                    Color(0xFF10b981) else Color(0xFFef4444),
                                shape = RoundedCornerShape(6.dp)
                            )
                    )
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = if (connectionState == RosbridgeClient.ConnectionState.CONNECTED)
                            "已连接到Rosbridge" else "未连接",
                        color = if (connectionState == RosbridgeClient.ConnectionState.CONNECTED)
                            Color(0xFF10b981) else Color(0xFFef4444),
                        fontSize = 16.sp,
                        fontWeight = FontWeight.SemiBold
                    )
                }
                
                Button(
                    onClick = onDisconnect,
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(top = 16.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFef4444)
                    ),
                    shape = RoundedCornerShape(12.dp)
                ) {
                    Text("断开连接", fontSize = 16.sp)
                }
            }
            
            Spacer(modifier = Modifier.height(24.dp))
            
            // ROS话题设置
            SettingsSection(title = "ROS话题设置") {
                SettingTextField(
                    label = "速度控制话题",
                    value = cmdVelTopicText,
                    onValueChange = { cmdVelTopicText = it },
                    onDone = { viewModel.saveCmdVelTopic(cmdVelTopicText) }
                )
                
                Spacer(modifier = Modifier.height(12.dp))
                
                SettingTextField(
                    label = "地图话题",
                    value = mapTopicText,
                    onValueChange = { mapTopicText = it },
                    onDone = { viewModel.saveMapTopic(mapTopicText) }
                )
                
                Spacer(modifier = Modifier.height(12.dp))
                
                SettingTextField(
                    label = "TF话题",
                    value = tfTopicText,
                    onValueChange = { tfTopicText = it },
                    onDone = { viewModel.saveTfTopic(tfTopicText) }
                )
                
                Spacer(modifier = Modifier.height(12.dp))
                
                SettingTextField(
                    label = "相机图像话题",
                    value = cameraTopicText,
                    onValueChange = { cameraTopicText = it },
                    onDone = { viewModel.saveCameraTopic(cameraTopicText) }
                )
            }
            
            Spacer(modifier = Modifier.height(24.dp))
            
            // 速度参数设置
            SettingsSection(title = "速度参数") {
                SettingTextField(
                    label = "最大线速度 (m/s)",
                    value = linearSpeedText,
                    onValueChange = { linearSpeedText = it },
                    onDone = {
                        linearSpeedText.toFloatOrNull()?.let {
                            viewModel.saveMaxLinearSpeed(it)
                        }
                    },
                    keyboardType = KeyboardType.Decimal
                )
                
                Spacer(modifier = Modifier.height(12.dp))
                
                SettingTextField(
                    label = "最大角速度 (rad/s)",
                    value = angularSpeedText,
                    onValueChange = { angularSpeedText = it },
                    onDone = {
                        angularSpeedText.toFloatOrNull()?.let {
                            viewModel.saveMaxAngularSpeed(it)
                        }
                    },
                    keyboardType = KeyboardType.Decimal
                )
            }
            
            Spacer(modifier = Modifier.height(24.dp))
            
            // 订阅频率设置
            SettingsSection(title = "订阅频率 (ms)") {
                SettingTextField(
                    label = "地图更新频率",
                    value = mapThrottleText,
                    onValueChange = { mapThrottleText = it },
                    onDone = {
                        mapThrottleText.toIntOrNull()?.let {
                            viewModel.saveMapThrottle(it)
                        }
                    },
                    keyboardType = KeyboardType.Number
                )
                
                Spacer(modifier = Modifier.height(12.dp))
                
                SettingTextField(
                    label = "TF更新频率",
                    value = tfThrottleText,
                    onValueChange = { tfThrottleText = it },
                    onDone = {
                        tfThrottleText.toIntOrNull()?.let {
                            viewModel.saveTfThrottle(it)
                        }
                    },
                    keyboardType = KeyboardType.Number
                )
            }
            
            Spacer(modifier = Modifier.height(32.dp))
        }
    }
}

@Composable
fun SettingsSection(
    title: String,
    content: @Composable ColumnScope.() -> Unit
) {
    Surface(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(16.dp),
        color = Color(0xFF374151)
    ) {
        Column(
            modifier = Modifier.padding(24.dp)
        ) {
            Text(
                text = title,
                fontSize = 20.sp,
                fontWeight = FontWeight.SemiBold,
                color = Color.White,
                modifier = Modifier.padding(bottom = 16.dp)
            )
            content()
        }
    }
}

@Composable
fun SettingTextField(
    label: String,
    value: String,
    onValueChange: (String) -> Unit,
    onDone: () -> Unit = {},
    keyboardType: KeyboardType = KeyboardType.Text
) {
    Column {
        Text(
            text = label,
            fontSize = 14.sp,
            fontWeight = FontWeight.Medium,
            color = Color.Gray,
            modifier = Modifier.padding(bottom = 8.dp)
        )
        
        OutlinedTextField(
            value = value,
            onValueChange = onValueChange,
            modifier = Modifier.fillMaxWidth(),
            colors = OutlinedTextFieldDefaults.colors(
                focusedBorderColor = Color(0xFF2563eb),
                unfocusedBorderColor = Color(0xFF4b5563),
                focusedTextColor = Color.White,
                unfocusedTextColor = Color.White,
                cursorColor = Color.White
            ),
            shape = RoundedCornerShape(8.dp),
            keyboardOptions = KeyboardOptions(keyboardType = keyboardType),
            singleLine = true,
            trailingIcon = {
                TextButton(
                    onClick = onDone,
                    colors = ButtonDefaults.textButtonColors(
                        contentColor = Color(0xFF2563eb)
                    )
                ) {
                    Text("保存", fontSize = 14.sp)
                }
            }
        )
    }
}
