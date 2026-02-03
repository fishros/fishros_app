import { useEffect, useRef, useState } from 'react';
import { ArrowLeft } from 'lucide-react';
import { useRosbridge } from './RosbridgeContext';
import { VirtualJoystick } from './VirtualJoystick';
import { MapDisplay } from './MapDisplay';

interface ControlPanelProps {
  onBack: () => void;
}

export function ControlPanel({ onBack }: ControlPanelProps) {
  const { publishMessage } = useRosbridge();
  const [linearVelocity, setLinearVelocity] = useState(0);
  const [angularVelocity, setAngularVelocity] = useState(0);
  const publishIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // 持续发布cmd_vel消息
  useEffect(() => {
    if (publishIntervalRef.current) {
      clearInterval(publishIntervalRef.current);
    }

    publishIntervalRef.current = setInterval(() => {
      publishCmdVel(linearVelocity, angularVelocity);
    }, 100); // 10Hz发布频率

    return () => {
      if (publishIntervalRef.current) {
        clearInterval(publishIntervalRef.current);
      }
      // 停止时发送零速度
      publishCmdVel(0, 0);
    };
  }, [linearVelocity, angularVelocity]);

  const publishCmdVel = (linear: number, angular: number) => {
    const message = {
      linear: {
        x: linear,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angular
      }
    };
    publishMessage('/cmd_vel', 'geometry_msgs/Twist', message);
  };

  const handleLinearChange = (x: number, y: number) => {
    // y轴控制前进后退，向上为正（前进），向下为负（后退）
    setLinearVelocity(y * 0.5); // 最大速度0.5 m/s
  };

  const handleAngularChange = (x: number, y: number) => {
    // x轴控制左右转，向左为正，向右为负
    setAngularVelocity(x * 1.0); // 最大角速度1.0 rad/s
  };

  return (
    <div className="w-full h-full relative bg-gray-900">
      {/* 返回按钮 */}
      <button
        onClick={onBack}
        className="absolute top-4 left-4 z-20 p-3 bg-gray-800/90 hover:bg-gray-700 text-white rounded-lg shadow-lg transition-colors duration-200 flex items-center gap-2"
      >
        <ArrowLeft className="w-5 h-5" />
        <span>返回</span>
      </button>

      {/* 速度显示 */}
      <div className="absolute top-4 right-4 z-20 bg-gray-800/90 text-white rounded-lg shadow-lg p-4 min-w-[200px]">
        <div className="text-sm space-y-1">
          <div className="flex justify-between">
            <span className="text-gray-400">线速度:</span>
            <span className="font-mono">{linearVelocity.toFixed(2)} m/s</span>
          </div>
          <div className="flex justify-between">
            <span className="text-gray-400">角速度:</span>
            <span className="font-mono">{angularVelocity.toFixed(2)} rad/s</span>
          </div>
        </div>
      </div>

      {/* 地图显示区域 */}
      <MapDisplay />

      {/* 左下角 - 前进后退摇杆 */}
      <div className="absolute bottom-8 left-8 z-10">
        <div className="mb-2 text-center text-white text-sm bg-gray-800/80 px-3 py-1 rounded">
          前进/后退
        </div>
        <VirtualJoystick
          size={160}
          onChange={handleLinearChange}
          mode="vertical"
        />
      </div>

      {/* 右下角 - 左转右转摇杆 */}
      <div className="absolute bottom-8 right-8 z-10">
        <div className="mb-2 text-center text-white text-sm bg-gray-800/80 px-3 py-1 rounded">
          左转/右转
        </div>
        <VirtualJoystick
          size={160}
          onChange={handleAngularChange}
          mode="horizontal"
        />
      </div>
    </div>
  );
}
