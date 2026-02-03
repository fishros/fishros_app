#!/bin/bash

# ROS Car Android App - 快速构建脚本

echo "======================================"
echo "ROS Car Android App Builder"
echo "======================================"
echo ""

# 检查Gradle
if [ ! -f "./gradlew" ]; then
    echo "❌ 错误: 找不到gradlew文件"
    echo "请在项目根目录运行此脚本"
    exit 1
fi

# 赋予执行权限
chmod +x ./gradlew

echo "📦 正在清理旧构建..."
./gradlew clean

echo ""
echo "🔨 正在构建项目..."
./gradlew assembleDebug

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 构建成功！"
    echo ""
    echo "APK位置: app/build/outputs/apk/debug/app-debug.apk"
    echo ""
    
    # 检查是否有连接的设备
    if command -v adb &> /dev/null; then
        DEVICES=$(adb devices | grep -w device | wc -l)
        if [ $DEVICES -gt 0 ]; then
            echo "📱 检测到Android设备，是否安装？(y/n)"
            read -r response
            if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
                echo "正在安装..."
                ./gradlew installDebug
                if [ $? -eq 0 ]; then
                    echo "✅ 安装成功！"
                    echo ""
                    echo "🚀 使用步骤："
                    echo "1. 确保ROS端已启动rosbridge: roslaunch rosbridge_server rosbridge_websocket.launch"
                    echo "2. 在手机上打开ROS Car应用"
                    echo "3. 输入ROS主机的IP地址（例如: 192.168.1.100）"
                    echo "4. 端口号默认9090"
                    echo "5. 点击连接"
                else
                    echo "❌ 安装失败"
                fi
            fi
        else
            echo "ℹ️  未检测到Android设备"
            echo "可以手动安装APK: app/build/outputs/apk/debug/app-debug.apk"
        fi
    fi
else
    echo ""
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi

echo ""
echo "======================================"
echo "构建完成"
echo "======================================"
