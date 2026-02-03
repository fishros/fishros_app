import { useState } from 'react';
import { ArrowLeft, Wifi, WifiOff } from 'lucide-react';
import { useRosbridge } from './RosbridgeContext';

interface SettingsPageProps {
  onBack: () => void;
  onDisconnect: () => void;
}

export function SettingsPage({ onBack, onDisconnect }: SettingsPageProps) {
  const { isConnected } = useRosbridge();
  const [cmdVelTopic, setCmdVelTopic] = useState('/cmd_vel');
  const [mapTopic, setMapTopic] = useState('/map');
  const [tfTopic, setTfTopic] = useState('/tf');
  const [maxLinearSpeed, setMaxLinearSpeed] = useState(0.5);
  const [maxAngularSpeed, setMaxAngularSpeed] = useState(1.0);

  const handleSave = () => {
    // 这里可以保存设置到localStorage
    localStorage.setItem('ros_settings', JSON.stringify({
      cmdVelTopic,
      mapTopic,
      tfTopic,
      maxLinearSpeed,
      maxAngularSpeed
    }));
    alert('设置已保存');
  };

  return (
    <div className="w-full h-full bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900 overflow-auto">
      <div className="min-h-full p-8">
        {/* 返回按钮 */}
        <button
          onClick={onBack}
          className="mb-8 p-3 bg-gray-800 hover:bg-gray-700 text-white rounded-lg shadow-lg transition-colors duration-200 flex items-center gap-2"
        >
          <ArrowLeft className="w-5 h-5" />
          <span>返回</span>
        </button>

        <div className="max-w-2xl mx-auto">
          <h1 className="text-4xl font-bold text-white mb-8">设置</h1>

          {/* 连接状态 */}
          <div className="bg-gray-800 rounded-lg p-6 mb-6">
            <h2 className="text-xl font-semibold text-white mb-4">连接状态</h2>
            <div className="flex items-center gap-3">
              {isConnected ? (
                <>
                  <Wifi className="w-6 h-6 text-green-500" />
                  <span className="text-green-500 font-semibold">已连接到Rosbridge</span>
                </>
              ) : (
                <>
                  <WifiOff className="w-6 h-6 text-red-500" />
                  <span className="text-red-500 font-semibold">未连接</span>
                </>
              )}
            </div>
            <button
              onClick={onDisconnect}
              className="mt-4 px-6 py-2 bg-red-600 hover:bg-red-700 text-white rounded-lg transition-colors duration-200"
            >
              断开连接
            </button>
          </div>

          {/* ROS话题设置 */}
          <div className="bg-gray-800 rounded-lg p-6 mb-6">
            <h2 className="text-xl font-semibold text-white mb-4">ROS话题设置</h2>
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">
                  速度控制话题
                </label>
                <input
                  type="text"
                  value={cmdVelTopic}
                  onChange={(e) => setCmdVelTopic(e.target.value)}
                  className="w-full px-4 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white focus:outline-none focus:border-blue-500"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">
                  地图话题
                </label>
                <input
                  type="text"
                  value={mapTopic}
                  onChange={(e) => setMapTopic(e.target.value)}
                  className="w-full px-4 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white focus:outline-none focus:border-blue-500"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">
                  TF话题
                </label>
                <input
                  type="text"
                  value={tfTopic}
                  onChange={(e) => setTfTopic(e.target.value)}
                  className="w-full px-4 py-2 bg-gray-700 border border-gray-600 rounded-lg text-white focus:outline-none focus:border-blue-500"
                />
              </div>
            </div>
          </div>

          {/* 速度限制设置 */}
          <div className="bg-gray-800 rounded-lg p-6 mb-6">
            <h2 className="text-xl font-semibold text-white mb-4">速度限制</h2>
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">
                  最大线速度 (m/s): {maxLinearSpeed.toFixed(2)}
                </label>
                <input
                  type="range"
                  min="0.1"
                  max="2.0"
                  step="0.1"
                  value={maxLinearSpeed}
                  onChange={(e) => setMaxLinearSpeed(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-300 mb-2">
                  最大角速度 (rad/s): {maxAngularSpeed.toFixed(2)}
                </label>
                <input
                  type="range"
                  min="0.1"
                  max="3.0"
                  step="0.1"
                  value={maxAngularSpeed}
                  onChange={(e) => setMaxAngularSpeed(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>
            </div>
          </div>

          {/* 系统信息 */}
          <div className="bg-gray-800 rounded-lg p-6 mb-6">
            <h2 className="text-xl font-semibold text-white mb-4">系统信息</h2>
            <div className="space-y-2 text-gray-300">
              <div className="flex justify-between">
                <span>版本:</span>
                <span className="font-mono">v1.0.0</span>
              </div>
              <div className="flex justify-between">
                <span>屏幕尺寸:</span>
                <span className="font-mono">{window.innerWidth} × {window.innerHeight}</span>
              </div>
              <div className="flex justify-between">
                <span>用户代理:</span>
                <span className="font-mono text-xs truncate max-w-xs">{navigator.userAgent.split(' ')[0]}</span>
              </div>
            </div>
          </div>

          {/* 保存按钮 */}
          <button
            onClick={handleSave}
            className="w-full py-3 bg-blue-600 hover:bg-blue-700 text-white font-semibold rounded-lg transition-colors duration-200"
          >
            保存设置
          </button>
        </div>
      </div>
    </div>
  );
}
