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
fun ConnectionScreen(
    viewModel: RosViewModel,
    onConnectionSuccess: () -> Unit
) {
    val connectionState by viewModel.connectionState.collectAsState()
    val errorMessage by viewModel.errorMessage.collectAsState()
    val lastIp by viewModel.lastIp.collectAsState()
    val lastPort by viewModel.lastPort.collectAsState()
    
    var ip by remember { mutableStateOf(lastIp) }
    var port by remember { mutableStateOf(lastPort) }
    
    // 当上次的值更新时同步（只在首次加载时）
    LaunchedEffect(lastIp) {
        if (ip.isEmpty() || ip == "192.168.1.100") {
            ip = lastIp
        }
    }
    
    LaunchedEffect(lastPort) {
        if (port.isEmpty() || port == "9090") {
            port = lastPort
        }
    }
    
    // 监听连接状态
    LaunchedEffect(connectionState) {
        if (connectionState == RosbridgeClient.ConnectionState.CONNECTED) {
            onConnectionSuccess()
        }
    }
    
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
                .padding(32.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            // Logo区域
            Surface(
                modifier = Modifier.size(80.dp),
                shape = RoundedCornerShape(16.dp),
                color = Color(0xFF2563eb)
            ) {
                Box(
                    modifier = Modifier.fillMaxSize(),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "📡",
                        fontSize = 40.sp
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(24.dp))
            
            Text(
                text = "ROS控制台",
                fontSize = 36.sp,
                fontWeight = FontWeight.Bold,
                color = Color.White
            )
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Text(
                text = "连接到Rosbridge服务器",
                fontSize = 16.sp,
                color = Color.Gray
            )
            
            Spacer(modifier = Modifier.height(48.dp))
            
            // IP输入框
            OutlinedTextField(
                value = ip,
                onValueChange = { ip = it },
                label = { Text("IP地址") },
                placeholder = { Text("192.168.1.100") },
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 64.dp),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedBorderColor = Color(0xFF2563eb),
                    unfocusedBorderColor = Color.Gray,
                    focusedTextColor = Color.White,
                    unfocusedTextColor = Color.White,
                    cursorColor = Color.White,
                    focusedLabelColor = Color(0xFF2563eb),
                    unfocusedLabelColor = Color.Gray
                ),
                singleLine = true
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // 端口输入框
            OutlinedTextField(
                value = port,
                onValueChange = { port = it },
                label = { Text("端口") },
                placeholder = { Text("9090") },
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 64.dp),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedBorderColor = Color(0xFF2563eb),
                    unfocusedBorderColor = Color.Gray,
                    focusedTextColor = Color.White,
                    unfocusedTextColor = Color.White,
                    cursorColor = Color.White,
                    focusedLabelColor = Color(0xFF2563eb),
                    unfocusedLabelColor = Color.Gray
                ),
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                singleLine = true
            )
            
            Spacer(modifier = Modifier.height(32.dp))
            
            // 连接按钮
            Button(
                onClick = {
                    if (ip.isNotEmpty() && port.isNotEmpty()) {
                        viewModel.connect(ip, port)
                    }
                },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(56.dp)
                    .padding(horizontal = 64.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF2563eb),
                    disabledContainerColor = Color.Gray
                ),
                enabled = connectionState != RosbridgeClient.ConnectionState.CONNECTING,
                shape = RoundedCornerShape(12.dp)
            ) {
                if (connectionState == RosbridgeClient.ConnectionState.CONNECTING) {
                    CircularProgressIndicator(
                        modifier = Modifier.size(24.dp),
                        color = Color.White
                    )
                    Spacer(modifier = Modifier.width(12.dp))
                    Text("连接中...", fontSize = 18.sp)
                } else {
                    Text("连接", fontSize = 18.sp, fontWeight = FontWeight.Bold)
                }
            }
            
            // 错误信息
            if (errorMessage != null) {
                Spacer(modifier = Modifier.height(16.dp))
                Text(
                    text = errorMessage ?: "",
                    color = Color(0xFFef4444),
                    fontSize = 14.sp
                )
            }
            
            // 状态指示
            Spacer(modifier = Modifier.height(24.dp))
            
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.Center
            ) {
                Box(
                    modifier = Modifier
                        .size(12.dp)
                        .background(
                            color = when (connectionState) {
                                RosbridgeClient.ConnectionState.CONNECTED -> Color(0xFF10b981)
                                RosbridgeClient.ConnectionState.CONNECTING -> Color(0xFFfbbf24)
                                RosbridgeClient.ConnectionState.FAILED -> Color(0xFFef4444)
                                else -> Color.Gray
                            },
                            shape = RoundedCornerShape(6.dp)
                        )
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = when (connectionState) {
                        RosbridgeClient.ConnectionState.CONNECTED -> "已连接"
                        RosbridgeClient.ConnectionState.CONNECTING -> "连接中"
                        RosbridgeClient.ConnectionState.FAILED -> "连接失败"
                        else -> "未连接"
                    },
                    color = Color.Gray,
                    fontSize = 14.sp
                )
            }
        }
    }
}
