package com.example.ros_car.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun HomeScreen(
    onNavigateToControl: () -> Unit,
    onNavigateToSettings: () -> Unit
) {
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
                .padding(32.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            // 状态指示
            Row(
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.padding(bottom = 32.dp)
            ) {
                Box(
                    modifier = Modifier
                        .size(12.dp)
                        .background(
                            color = Color(0xFF10b981),
                            shape = RoundedCornerShape(6.dp)
                        )
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "已连接",
                    color = Color(0xFF10b981),
                    fontSize = 16.sp,
                    fontWeight = FontWeight.SemiBold
                )
            }
            
            Text(
                text = "控制中心",
                fontSize = 48.sp,
                fontWeight = FontWeight.Bold,
                color = Color.White
            )
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Text(
                text = "选择功能进入",
                fontSize = 16.sp,
                color = Color.Gray
            )
            
            Spacer(modifier = Modifier.height(64.dp))
            
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 80.dp),
                horizontalArrangement = Arrangement.spacedBy(24.dp)
            ) {
                // 主控界面卡片
                FeatureCard(
                    title = "主控界面",
                    description = "机器人控制与地图显示",
                    icon = "🎮",
                    gradient = Brush.linearGradient(
                        colors = listOf(
                            Color(0xFF2563eb),
                            Color(0xFF1e40af)
                        )
                    ),
                    modifier = Modifier.weight(1f),
                    onClick = onNavigateToControl
                )
                
                // 设置卡片
                FeatureCard(
                    title = "设置",
                    description = "系统配置与连接管理",
                    icon = "⚙️",
                    gradient = Brush.linearGradient(
                        colors = listOf(
                            Color(0xFF4b5563),
                            Color(0xFF1f2937)
                        )
                    ),
                    modifier = Modifier.weight(1f),
                    onClick = onNavigateToSettings
                )
            }
        }
    }
}

@Composable
fun FeatureCard(
    title: String,
    description: String,
    icon: String,
    gradient: Brush,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Surface(
        modifier = modifier
            .height(200.dp)
            .clickable(onClick = onClick),
        shape = RoundedCornerShape(20.dp),
        shadowElevation = 8.dp
    ) {
        Box(
            modifier = Modifier
                .fillMaxSize()
                .background(gradient)
                .padding(24.dp)
        ) {
            Column(
                modifier = Modifier.fillMaxSize(),
                verticalArrangement = Arrangement.Center,
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Surface(
                    modifier = Modifier.size(64.dp),
                    shape = RoundedCornerShape(12.dp),
                    color = Color.White.copy(alpha = 0.1f)
                ) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = icon,
                            fontSize = 32.sp
                        )
                    }
                }
                
                Spacer(modifier = Modifier.height(16.dp))
                
                Text(
                    text = title,
                    fontSize = 22.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.White
                )
                
                Spacer(modifier = Modifier.height(6.dp))
                
                Text(
                    text = description,
                    fontSize = 12.sp,
                    color = Color.White.copy(alpha = 0.8f)
                )
            }
        }
    }
}
