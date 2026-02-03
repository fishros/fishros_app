import { useEffect, useRef, useState } from 'react';

interface VirtualJoystickProps {
  size?: number;
  onChange: (x: number, y: number) => void;
  mode?: 'both' | 'horizontal' | 'vertical';
}

export function VirtualJoystick({ size = 150, onChange, mode = 'both' }: VirtualJoystickProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const centerRef = useRef({ x: size / 2, y: size / 2 });
  const maxDistanceRef = useRef(size / 3);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // 清空画布
    ctx.clearRect(0, 0, size, size);

    // 绘制外圈
    ctx.beginPath();
    ctx.arc(centerRef.current.x, centerRef.current.y, maxDistanceRef.current, 0, 2 * Math.PI);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.fill();

    // 绘制中心点
    ctx.beginPath();
    ctx.arc(centerRef.current.x, centerRef.current.y, 4, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
    ctx.fill();

    // 绘制摇杆
    const joystickX = centerRef.current.x + position.x * maxDistanceRef.current;
    const joystickY = centerRef.current.y + position.y * maxDistanceRef.current;

    ctx.beginPath();
    ctx.arc(joystickX, joystickY, 35, 0, 2 * Math.PI);
    ctx.fillStyle = isDragging ? 'rgba(59, 130, 246, 0.9)' : 'rgba(59, 130, 246, 0.7)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.lineWidth = 3;
    ctx.stroke();

    // 绘制方向指示线
    if (position.x !== 0 || position.y !== 0) {
      ctx.beginPath();
      ctx.moveTo(centerRef.current.x, centerRef.current.y);
      ctx.lineTo(joystickX, joystickY);
      ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [position, isDragging, size]);

  const handleStart = (clientX: number, clientY: number) => {
    setIsDragging(true);
    handleMove(clientX, clientY);
  };

  const handleMove = (clientX: number, clientY: number) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = clientX - rect.left - centerRef.current.x;
    const y = clientY - rect.top - centerRef.current.y;

    let distance = Math.sqrt(x * x + y * y);
    let normalizedX = x / maxDistanceRef.current;
    let normalizedY = y / maxDistanceRef.current;

    // 限制在圆形区域内
    if (distance > maxDistanceRef.current) {
      const angle = Math.atan2(y, x);
      normalizedX = Math.cos(angle);
      normalizedY = Math.sin(angle);
    }

    // 根据模式限制移动方向
    if (mode === 'horizontal') {
      normalizedY = 0;
    } else if (mode === 'vertical') {
      normalizedX = 0;
    }

    setPosition({ x: normalizedX, y: -normalizedY }); // Y轴反转，向上为正
    onChange(normalizedX, -normalizedY);
  };

  const handleEnd = () => {
    setIsDragging(false);
    setPosition({ x: 0, y: 0 });
    onChange(0, 0);
  };

  const handleTouchStart = (e: React.TouchEvent) => {
    e.preventDefault();
    const touch = e.touches[0];
    handleStart(touch.clientX, touch.clientY);
  };

  const handleTouchMove = (e: React.TouchEvent) => {
    e.preventDefault();
    if (!isDragging) return;
    const touch = e.touches[0];
    handleMove(touch.clientX, touch.clientY);
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    handleStart(e.clientX, e.clientY);
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    e.preventDefault();
    if (!isDragging) return;
    handleMove(e.clientX, e.clientY);
  };

  useEffect(() => {
    const handleGlobalMouseUp = () => {
      if (isDragging) {
        handleEnd();
      }
    };

    const handleGlobalTouchEnd = () => {
      if (isDragging) {
        handleEnd();
      }
    };

    window.addEventListener('mouseup', handleGlobalMouseUp);
    window.addEventListener('touchend', handleGlobalTouchEnd);

    return () => {
      window.removeEventListener('mouseup', handleGlobalMouseUp);
      window.removeEventListener('touchend', handleGlobalTouchEnd);
    };
  }, [isDragging]);

  return (
    <canvas
      ref={canvasRef}
      width={size}
      height={size}
      className="touch-none cursor-pointer select-none"
      onTouchStart={handleTouchStart}
      onTouchMove={handleTouchMove}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      style={{
        border: '2px solid rgba(255, 255, 255, 0.2)',
        borderRadius: '50%',
        backgroundColor: 'rgba(0, 0, 0, 0.5)'
      }}
    />
  );
}
