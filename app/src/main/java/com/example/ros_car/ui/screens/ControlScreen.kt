package com.example.ros_car.ui.screens

import android.graphics.Bitmap
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.ros_car.viewmodel.RosViewModel
import kotlin.math.cos
import kotlin.math.sin
import androidx.compose.ui.text.font.FontWeight
import com.example.ros_car.ui.components.VirtualJoystick

@Composable
fun ControlScreen(
    viewModel: RosViewModel,
    onBack: () -> Unit
) {
    val mapBitmap by viewModel.mapBitmap.collectAsState()
    val mapMetadata by viewModel.mapMetadata.collectAsState()
    val robotPose by viewModel.robotPose.collectAsState()
    val maxLinearSpeed by viewModel.maxLinearSpeed.collectAsState()
    val maxAngularSpeed by viewModel.maxAngularSpeed.collectAsState()
    val cameraBitmap by viewModel.cameraBitmap.collectAsState()
    val cameraEnabled by viewModel.cameraEnabled.collectAsState()
    
    var currentLinearX by remember { mutableStateOf(0f) }
    var currentLinearY by remember { mutableStateOf(0f) }
    var currentAngularZ by remember { mutableStateOf(0f) }
    
    // 订阅topics
    LaunchedEffect(Unit) {
        viewModel.subscribeTopics()
    }
    
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF1a1a1a))
    ) {
        // 地图背景
        MapView(
            bitmap = mapBitmap,
            metadata = mapMetadata,
            robotPose = robotPose,
            modifier = Modifier.fillMaxSize()
        )
        
        // 返回按钮
        IconButton(
            onClick = onBack,
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(16.dp)
        ) {
            Surface(
                shape = RoundedCornerShape(12.dp),
                color = Color.Black.copy(alpha = 0.5f),
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
        
        // 机器人位姿信息
        Surface(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(start = 80.dp, top = 16.dp),
            shape = RoundedCornerShape(12.dp),
            color = Color.Black.copy(alpha = 0.5f)
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Text(
                    "位姿信息",
                    color = Color.White,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold
                )
                Spacer(modifier = Modifier.height(4.dp))
                Text(
                    "X: %.2f m".format(robotPose.x),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
                Text(
                    "Y: %.2f m".format(robotPose.y),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
                Text(
                    "θ: %.2f°".format(Math.toDegrees(robotPose.theta)),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
            }
        }
        
        // 速度信息显示
        Surface(
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(16.dp),
            shape = RoundedCornerShape(12.dp),
            color = Color.Black.copy(alpha = 0.5f)
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Text(
                    "速度控制",
                    color = Color.White,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold
                )
                Spacer(modifier = Modifier.height(4.dp))
                Text(
                    "VX: %.2f".format(currentLinearX),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
                Text(
                    "VY: %.2f".format(currentLinearY),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
                Text(
                    "ωZ: %.2f".format(currentAngularZ),
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 12.sp
                )
            }
        }
        
        // 相机画面显示 (右上角，速度信息下方)
        if (cameraEnabled) {
            Column(
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(end = 16.dp, top = 160.dp)
            ) {
                Surface(
                    modifier = Modifier
                        .width(200.dp)
                        .height(150.dp),
                    shape = RoundedCornerShape(12.dp),
                    color = Color.Black.copy(alpha = 0.7f)
                ) {
                    Box(
                        modifier = Modifier.fillMaxSize()
                    ) {
                        if (cameraBitmap != null) {
                            Image(
                                bitmap = cameraBitmap!!.asImageBitmap(),
                                contentDescription = "相机画面",
                                modifier = Modifier
                                    .fillMaxSize()
                                    .clip(RoundedCornerShape(12.dp)),
                                contentScale = ContentScale.Fit
                            )
                        } else {
                            Box(
                                modifier = Modifier.fillMaxSize(),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    "等待图像...",
                                    color = Color.White.copy(alpha = 0.6f),
                                    fontSize = 12.sp
                                )
                            }
                        }
                        
                        // 关闭按钮
                        IconButton(
                            onClick = { viewModel.toggleCamera(false) },
                            modifier = Modifier
                                .align(Alignment.TopEnd)
                                .padding(4.dp)
                                .size(24.dp)
                        ) {
                            Surface(
                                shape = CircleShape,
                                color = Color.Black.copy(alpha = 0.6f),
                                modifier = Modifier.fillMaxSize()
                            ) {
                                Box(
                                    modifier = Modifier.fillMaxSize(),
                                    contentAlignment = Alignment.Center
                                ) {
                                    Text(
                                        "✕",
                                        color = Color.White,
                                        fontSize = 14.sp,
                                        fontWeight = FontWeight.Bold
                                    )
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // 开启相机按钮
            IconButton(
                onClick = { viewModel.toggleCamera(true) },
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(end = 16.dp, top = 160.dp)
            ) {
                Surface(
                    shape = RoundedCornerShape(12.dp),
                    color = Color(0xFF2563eb).copy(alpha = 0.8f),
                    modifier = Modifier.size(48.dp)
                ) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            "📷",
                            fontSize = 24.sp
                        )
                    }
                }
            }
        }
        
        Row(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(32.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.Bottom
        ) {
            // 左侧：线速度摇杆（X,Y）
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Text(
                    "线速度 (X/Y)",
                    color = Color.White,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(bottom = 8.dp)
                )
                VirtualJoystick(
                    size = 180f,
                    onMove = { x, y ->
                        // 摇杆x->机器人y (反向), 摇杆y->机器人x
                        currentLinearX = y * maxLinearSpeed
                        currentLinearY = -x * maxLinearSpeed  // Y轴反向
                        viewModel.setVelocityXYZ(
                            linearX = (y * maxLinearSpeed).toDouble(),
                            linearY = (-x * maxLinearSpeed).toDouble(),  // Y轴反向
                            angularZ = currentAngularZ.toDouble()
                        )
                    },
                    onRelease = {
                        currentLinearX = 0f
                        currentLinearY = 0f
                        viewModel.setVelocityXYZ(0.0, 0.0, currentAngularZ.toDouble())
                    }
                )
            }
            
            // 右侧：角速度摇杆（Z）
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Text(
                    "角速度 (Z)",
                    color = Color.White,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(bottom = 8.dp)
                )
                VirtualJoystick(
                    size = 180f,
                    onMove = { x, _ ->
                        // 只使用X轴控制角速度，反向
                        currentAngularZ = -x * maxAngularSpeed  // Z轴反向
                        viewModel.setVelocityXYZ(
                            linearX = currentLinearX.toDouble(),
                            linearY = currentLinearY.toDouble(),
                            angularZ = (-x * maxAngularSpeed).toDouble()  // Z轴反向
                        )
                    },
                    onRelease = {
                        currentAngularZ = 0f
                        viewModel.setVelocityXYZ(
                            currentLinearX.toDouble(),
                            currentLinearY.toDouble(),
                            0.0
                        )
                    }
                )
            }
        }
    }
}

@Composable
fun MapView(
    bitmap: Bitmap?,
    metadata: com.example.ros_car.data.MapMetadata?,
    robotPose: com.example.ros_car.rosbridge.RobotPose,
    modifier: Modifier = Modifier
) {
    var scale by remember { mutableStateOf(1f) }
    var offset by remember { mutableStateOf(Offset.Zero) }
    
    Canvas(
        modifier = modifier
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    scale = (scale * zoom).coerceIn(0.5f, 5f)
                    offset += pan
                }
            }
    ) {
        val canvasWidth = size.width
        val canvasHeight = size.height
        
        // 绘制背景
        drawRect(
            color = Color(0xFF2d2d2d),
            size = size
        )
        
        if (bitmap != null && metadata != null) {
            // 绘制地图
            val bitmapWidth = bitmap.width.toFloat()
            val bitmapHeight = bitmap.height.toFloat()
            
            // 计算居中位置
            val mapScale = scale * 2
            val scaledWidth = bitmapWidth * mapScale
            val scaledHeight = bitmapHeight * mapScale
            val centerX = canvasWidth / 2 + offset.x
            val centerY = canvasHeight / 2 + offset.y
            
            // 绘制地图
            drawContext.canvas.save()
            drawContext.canvas.translate(
                centerX - scaledWidth / 2,
                centerY - scaledHeight / 2
            )
            drawContext.canvas.scale(mapScale, mapScale)
            drawImage(
                image = bitmap.asImageBitmap(),
                topLeft = Offset.Zero,
                alpha = 0.8f
            )
            drawContext.canvas.restore()
            
            // 绘制机器人
            drawRobot(
                pose = robotPose,
                metadata = metadata,
                centerX = centerX,
                centerY = centerY,
                mapScale = mapScale,
                mapWidth = bitmapWidth,
                mapHeight = bitmapHeight
            )
        } else {
            // 无地图时显示提示
            drawContext.canvas.nativeCanvas.apply {
                val paint = android.graphics.Paint().apply {
                    color = android.graphics.Color.WHITE
                    textSize = 48f
                    textAlign = android.graphics.Paint.Align.CENTER
                }
                drawText(
                    "等待地图数据...",
                    canvasWidth / 2,
                    canvasHeight / 2,
                    paint
                )
            }
        }
    }
}

fun androidx.compose.ui.graphics.drawscope.DrawScope.drawRobot(
    pose: com.example.ros_car.rosbridge.RobotPose,
    metadata: com.example.ros_car.data.MapMetadata,
    centerX: Float,
    centerY: Float,
    mapScale: Float,
    mapWidth: Float,
    mapHeight: Float
) {
    // 将机器人世界坐标转换为地图像素坐标
    val robotMapX = ((pose.x - metadata.originX) / metadata.resolution).toFloat()
    val robotMapY = ((pose.y - metadata.originY) / metadata.resolution).toFloat()
    
    // 将地图像素坐标转换为屏幕坐标
    // 注意：地图已经在createMapBitmap中翻转了Y轴，所以这里直接使用robotMapY
    val robotScreenX = centerX - (mapWidth * mapScale / 2) + (robotMapX * mapScale)
    val robotScreenY = centerY - (mapHeight * mapScale / 2) + ((mapHeight - robotMapY) * mapScale)
    
    // 绘制机器人圆形
    drawCircle(
        color = Color(0xFF10b981),
        radius = 20f,
        center = Offset(robotScreenX, robotScreenY),
        style = Stroke(width = 3f)
    )
    
    drawCircle(
        color = Color(0xFF10b981).copy(alpha = 0.3f),
        radius = 20f,
        center = Offset(robotScreenX, robotScreenY)
    )
    
    // 绘制方向箭头
    rotate(
        degrees = -Math.toDegrees(pose.theta).toFloat(),
        pivot = Offset(robotScreenX, robotScreenY)
    ) {
        val arrowLength = 30f
        drawLine(
            color = Color(0xFF10b981),
            start = Offset(robotScreenX, robotScreenY),
            end = Offset(robotScreenX + arrowLength, robotScreenY),
            strokeWidth = 3f,
            cap = StrokeCap.Round
        )
        
        // 箭头头部
        drawLine(
            color = Color(0xFF10b981),
            start = Offset(robotScreenX + arrowLength, robotScreenY),
            end = Offset(robotScreenX + arrowLength - 10f, robotScreenY - 8f),
            strokeWidth = 3f,
            cap = StrokeCap.Round
        )
        drawLine(
            color = Color(0xFF10b981),
            start = Offset(robotScreenX + arrowLength, robotScreenY),
            end = Offset(robotScreenX + arrowLength - 10f, robotScreenY + 8f),
            strokeWidth = 3f,
            cap = StrokeCap.Round
        )
    }
}
