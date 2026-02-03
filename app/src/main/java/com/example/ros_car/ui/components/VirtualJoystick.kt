package com.example.ros_car.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import kotlin.math.sqrt
import kotlin.math.atan2

/**
 * 虚拟摇杆组件
 * @param size 摇杆大小
 * @param onMove 移动回调 (normalizedX: -1~1, normalizedY: -1~1)
 * @param onRelease 释放回调
 */
@Composable
fun VirtualJoystick(
    size: Float = 200f,
    onMove: (Float, Float) -> Unit,
    onRelease: () -> Unit,
    modifier: Modifier = Modifier
) {
    var dragOffset by remember { mutableStateOf(Offset.Zero) }
    val maxRadius = size / 2
    val handleRadius = size / 6

    Canvas(
        modifier = modifier
            .size(size.dp)
            .pointerInput(Unit) {
                detectDragGestures(
                    onDragStart = { offset ->
                        // 开始拖动
                        val centerOffset = offset - Offset(size * density / 2, size * density / 2)
                        dragOffset = limitToBounds(centerOffset, maxRadius * density)
                        
                        val normalizedX = dragOffset.x / (maxRadius * density)
                        val normalizedY = -dragOffset.y / (maxRadius * density) // Y轴反转
                        onMove(normalizedX, normalizedY)
                    },
                    onDrag = { change, _ ->
                        change.consume()
                        val centerOffset = change.position - Offset(size * density / 2, size * density / 2)
                        dragOffset = limitToBounds(centerOffset, maxRadius * density)
                        
                        val normalizedX = dragOffset.x / (maxRadius * density)
                        val normalizedY = -dragOffset.y / (maxRadius * density) // Y轴反转
                        onMove(normalizedX, normalizedY)
                    },
                    onDragEnd = {
                        dragOffset = Offset.Zero
                        onRelease()
                    },
                    onDragCancel = {
                        dragOffset = Offset.Zero
                        onRelease()
                    }
                )
            }
    ) {
        val centerX = size.dp.toPx() / 2
        val centerY = size.dp.toPx() / 2
        val maxRadiusPx = maxRadius.dp.toPx()
        val handleRadiusPx = handleRadius.dp.toPx()

        // 绘制外圈
        drawCircle(
            color = Color.White.copy(alpha = 0.2f),
            radius = maxRadiusPx,
            center = Offset(centerX, centerY),
            style = Stroke(width = 3.dp.toPx())
        )

        // 绘制十字线
        drawLine(
            color = Color.White.copy(alpha = 0.3f),
            start = Offset(centerX - maxRadiusPx, centerY),
            end = Offset(centerX + maxRadiusPx, centerY),
            strokeWidth = 1.dp.toPx()
        )
        drawLine(
            color = Color.White.copy(alpha = 0.3f),
            start = Offset(centerX, centerY - maxRadiusPx),
            end = Offset(centerX, centerY + maxRadiusPx),
            strokeWidth = 1.dp.toPx()
        )

        // 绘制内圈（活动区域）
        drawCircle(
            color = Color(0xFF2563eb).copy(alpha = 0.3f),
            radius = maxRadiusPx * 0.7f,
            center = Offset(centerX, centerY)
        )

        // 绘制手柄
        val handleX = centerX + dragOffset.x
        val handleY = centerY + dragOffset.y
        
        drawCircle(
            color = Color(0xFF2563eb),
            radius = handleRadiusPx,
            center = Offset(handleX, handleY)
        )
        
        drawCircle(
            color = Color.White.copy(alpha = 0.5f),
            radius = handleRadiusPx,
            center = Offset(handleX, handleY),
            style = Stroke(width = 2.dp.toPx())
        )

        // 如果有移动，绘制从中心到手柄的线
        if (dragOffset != Offset.Zero) {
            drawLine(
                color = Color(0xFF2563eb).copy(alpha = 0.5f),
                start = Offset(centerX, centerY),
                end = Offset(handleX, handleY),
                strokeWidth = 3.dp.toPx(),
                cap = StrokeCap.Round
            )
        }
    }
}

/**
 * 限制偏移量在圆形范围内
 */
private fun limitToBounds(offset: Offset, maxRadius: Float): Offset {
    val distance = sqrt(offset.x * offset.x + offset.y * offset.y)
    return if (distance > maxRadius) {
        val angle = atan2(offset.y, offset.x)
        Offset(
            x = maxRadius * kotlin.math.cos(angle),
            y = maxRadius * kotlin.math.sin(angle)
        )
    } else {
        offset
    }
}
