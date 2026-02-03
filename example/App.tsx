import { useState, useEffect } from 'react';
import { ConnectionPage } from './components/ConnectionPage';
import { HomePage } from './components/HomePage';
import { ControlPanel } from './components/ControlPanel';
import { SettingsPage } from './components/SettingsPage';
import { RosbridgeProvider } from './components/RosbridgeContext';

export default function App() {
  const [currentPage, setCurrentPage] = useState<'connection' | 'home' | 'control' | 'settings'>('connection');
  const [isConnected, setIsConnected] = useState(false);

  // 强制横屏提示
  useEffect(() => {
    const checkOrientation = () => {
      if (window.innerHeight > window.innerWidth) {
        // 竖屏时显示提示
        document.body.classList.add('portrait-mode');
      } else {
        document.body.classList.remove('portrait-mode');
      }
    };

    checkOrientation();
    window.addEventListener('resize', checkOrientation);
    window.addEventListener('orientationchange', checkOrientation);

    return () => {
      window.removeEventListener('resize', checkOrientation);
      window.removeEventListener('orientationchange', checkOrientation);
    };
  }, []);

  const handleConnectionSuccess = () => {
    setIsConnected(true);
    setCurrentPage('home');
  };

  const handleDisconnect = () => {
    setIsConnected(false);
    setCurrentPage('connection');
  };

  const handleNavigate = (page: 'home' | 'control' | 'settings') => {
    setCurrentPage(page);
  };

  return (
    <RosbridgeProvider>
      <div className="w-full h-screen overflow-hidden bg-gray-900">
        {/* 竖屏提示 */}
        <div className="portrait-warning fixed inset-0 bg-gray-900 flex items-center justify-center z-50 hidden">
          <div className="text-white text-center p-8">
            <div className="text-6xl mb-4">📱</div>
            <div className="text-2xl font-bold mb-2">请旋转设备</div>
            <div className="text-lg text-gray-400">此应用需要横屏使用</div>
          </div>
        </div>

        {/* 主要内容 */}
        {currentPage === 'connection' && (
          <ConnectionPage onConnectionSuccess={handleConnectionSuccess} />
        )}
        {currentPage === 'home' && (
          <HomePage onNavigate={handleNavigate} onDisconnect={handleDisconnect} />
        )}
        {currentPage === 'control' && (
          <ControlPanel onBack={() => handleNavigate('home')} />
        )}
        {currentPage === 'settings' && (
          <SettingsPage onBack={() => handleNavigate('home')} onDisconnect={handleDisconnect} />
        )}
      </div>
    </RosbridgeProvider>
  );
}
