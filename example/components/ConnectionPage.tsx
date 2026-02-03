import { useState } from 'react';
import { useRosbridge } from './RosbridgeContext';
import { Wifi } from 'lucide-react';

interface ConnectionPageProps {
  onConnectionSuccess: () => void;
}

export function ConnectionPage({ onConnectionSuccess }: ConnectionPageProps) {
  const [ip, setIp] = useState('192.168.1.100');
  const [port, setPort] = useState('9090');
  const [isConnecting, setIsConnecting] = useState(false);
  const [error, setError] = useState('');
  const { connect } = useRosbridge();

  const handleConnect = async () => {
    if (!ip || !port) {
      setError('请输入IP地址和端口号');
      return;
    }

    setIsConnecting(true);
    setError('');

    try {
      await connect(ip, port);
      onConnectionSuccess();
    } catch (err) {
      setError(err instanceof Error ? err.message : '连接失败，请检查IP和端口');
    } finally {
      setIsConnecting(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      handleConnect();
    }
  };

  return (
    <div className="w-full h-full flex items-center justify-center bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900">
      <div className="w-full max-w-md p-8">
        <div className="text-center mb-12">
          <div className="inline-block p-4 bg-blue-600 rounded-full mb-4">
            <Wifi className="w-12 h-12 text-white" />
          </div>
          <h1 className="text-4xl font-bold text-white mb-2">ROS控制台</h1>
          <p className="text-gray-400">连接到Rosbridge服务器</p>
        </div>

        <div className="space-y-6">
          <div>
            <label className="block text-sm font-medium text-gray-300 mb-2">
              IP地址
            </label>
            <input
              type="text"
              value={ip}
              onChange={(e) => setIp(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="192.168.1.100"
              className="w-full px-4 py-3 bg-gray-800 border border-gray-700 rounded-lg text-white placeholder-gray-500 focus:outline-none focus:border-blue-500 focus:ring-2 focus:ring-blue-500/50"
            />
          </div>

          <div>
            <label className="block text-sm font-medium text-gray-300 mb-2">
              端口号
            </label>
            <input
              type="text"
              value={port}
              onChange={(e) => setPort(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="9090"
              className="w-full px-4 py-3 bg-gray-800 border border-gray-700 rounded-lg text-white placeholder-gray-500 focus:outline-none focus:border-blue-500 focus:ring-2 focus:ring-blue-500/50"
            />
          </div>

          {error && (
            <div className="p-4 bg-red-900/50 border border-red-600 rounded-lg">
              <p className="text-red-300 text-sm">{error}</p>
            </div>
          )}

          <button
            onClick={handleConnect}
            disabled={isConnecting}
            className="w-full py-4 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white font-semibold rounded-lg transition-colors duration-200"
          >
            {isConnecting ? '连接中...' : '连接'}
          </button>
        </div>

        <div className="mt-8 text-center text-sm text-gray-500">
          <p>确保Rosbridge服务器正在运行</p>
          <p className="mt-1">默认端口: 9090</p>
        </div>
      </div>
    </div>
  );
}
