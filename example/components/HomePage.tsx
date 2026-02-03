import { Gamepad2, Settings, Wifi } from 'lucide-react';

interface HomePageProps {
  onNavigate: (page: 'control' | 'settings') => void;
  onDisconnect: () => void;
}

export function HomePage({ onNavigate, onDisconnect }: HomePageProps) {
  return (
    <div className="w-full h-full flex items-center justify-center bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900 p-8">
      <div className="w-full max-w-4xl">
        <div className="text-center mb-12">
          <div className="flex items-center justify-center gap-2 mb-4">
            <Wifi className="w-6 h-6 text-green-500" />
            <span className="text-green-500 font-semibold">已连接</span>
          </div>
          <h1 className="text-5xl font-bold text-white mb-2">控制中心</h1>
          <p className="text-gray-400">选择功能进入</p>
        </div>

        <div className="grid grid-cols-2 gap-8">
          {/* 主控界面 */}
          <button
            onClick={() => onNavigate('control')}
            className="group relative overflow-hidden bg-gradient-to-br from-blue-600 to-blue-800 hover:from-blue-500 hover:to-blue-700 rounded-2xl p-8 transition-all duration-300 transform hover:scale-105 shadow-2xl"
          >
            <div className="relative z-10">
              <div className="inline-block p-6 bg-white/10 rounded-full mb-6">
                <Gamepad2 className="w-16 h-16 text-white" />
              </div>
              <h2 className="text-3xl font-bold text-white mb-3">主控界面</h2>
              <p className="text-blue-100">机器人控制与地图显示</p>
            </div>
            <div className="absolute inset-0 bg-gradient-to-t from-black/20 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
          </button>

          {/* 设置 */}
          <button
            onClick={() => onNavigate('settings')}
            className="group relative overflow-hidden bg-gradient-to-br from-gray-700 to-gray-900 hover:from-gray-600 hover:to-gray-800 rounded-2xl p-8 transition-all duration-300 transform hover:scale-105 shadow-2xl"
          >
            <div className="relative z-10">
              <div className="inline-block p-6 bg-white/10 rounded-full mb-6">
                <Settings className="w-16 h-16 text-white" />
              </div>
              <h2 className="text-3xl font-bold text-white mb-3">设置</h2>
              <p className="text-gray-300">系统配置与连接管理</p>
            </div>
            <div className="absolute inset-0 bg-gradient-to-t from-black/20 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
          </button>
        </div>

        <div className="mt-12 text-center">
          <button
            onClick={onDisconnect}
            className="px-8 py-3 bg-red-600 hover:bg-red-700 text-white rounded-lg transition-colors duration-200"
          >
            断开连接
          </button>
        </div>
      </div>
    </div>
  );
}
