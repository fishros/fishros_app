import { useEffect, useRef, useState } from 'react';
import { useRosbridge } from './RosbridgeContext';

interface MapData {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

interface TFData {
  transforms: Array<{
    header: {
      frame_id: string;
    };
    child_frame_id: string;
    transform: {
      translation: { x: number; y: number; z: number };
      rotation: { x: number; y: number; z: number; w: number };
    };
  }>;
}

export function MapDisplay() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { subscribeTopic, unsubscribeTopic } = useRosbridge();
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [robotPose, setRobotPose] = useState({ x: 0, y: 0, theta: 0 });
  const [scale, setScale] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });

  useEffect(() => {
    // 订阅地图话题
    const mapSubId = subscribeTopic('/map', 'nav_msgs/OccupancyGrid', (msg: any) => {
      setMapData(msg);
    });

    // 订阅TF话题
    const tfSubId = subscribeTopic('/tf', 'tf2_msgs/TFMessage', (msg: TFData) => {
      // 查找base_link相对于map的变换
      const baseTransform = msg.transforms.find(
        (t) => t.child_frame_id === 'base_link' || t.child_frame_id === 'base_footprint'
      );

      if (baseTransform) {
        const { translation, rotation } = baseTransform.transform;
        // 从四元数计算yaw角
        const siny_cosp = 2 * (rotation.w * rotation.z + rotation.x * rotation.y);
        const cosy_cosp = 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z);
        const theta = Math.atan2(siny_cosp, cosy_cosp);

        setRobotPose({
          x: translation.x,
          y: translation.y,
          theta: theta
        });
      }
    });

    return () => {
      unsubscribeTopic(mapSubId);
      unsubscribeTopic(tfSubId);
    };
  }, [subscribeTopic, unsubscribeTopic]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;

    // 清空画布
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);

    if (mapData) {
      // 绘制栅格地图
      const { info, data } = mapData;
      const cellSize = Math.max(2, 5 * scale); // 根据缩放调整单元格大小

      // 居中显示
      const mapPixelWidth = info.width * cellSize;
      const mapPixelHeight = info.height * cellSize;
      const centerX = width / 2 - mapPixelWidth / 2 + offset.x;
      const centerY = height / 2 - mapPixelHeight / 2 + offset.y;

      ctx.save();
      ctx.translate(centerX, centerY);

      // 绘制栅格
      for (let y = 0; y < info.height; y++) {
        for (let x = 0; x < info.width; x++) {
          const index = y * info.width + x;
          const value = data[index];

          let color;
          if (value === -1) {
            // 未知区域
            color = '#2a2a2a';
          } else if (value === 0) {
            // 自由空间
            color = '#e0e0e0';
          } else {
            // 障碍物
            const intensity = Math.floor((value / 100) * 255);
            color = `rgb(${255 - intensity}, ${100 - intensity * 0.5}, ${100 - intensity * 0.5})`;
          }

          ctx.fillStyle = color;
          ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
        }
      }

      ctx.restore();
    }

    // 绘制机器人位置
    if (mapData) {
      const { info } = mapData;
      const cellSize = Math.max(2, 5 * scale);
      const mapPixelWidth = info.width * cellSize;
      const mapPixelHeight = info.height * cellSize;
      const centerX = width / 2 - mapPixelWidth / 2 + offset.x;
      const centerY = height / 2 - mapPixelHeight / 2 + offset.y;

      // 将机器人世界坐标转换为地图像素坐标
      const robotPixelX =
        centerX + (robotPose.x - info.origin.position.x) / info.resolution * cellSize;
      const robotPixelY =
        centerY + mapPixelHeight - (robotPose.y - info.origin.position.y) / info.resolution * cellSize;

      ctx.save();
      ctx.translate(robotPixelX, robotPixelY);
      ctx.rotate(-robotPose.theta); // 负号因为canvas的Y轴向下

      // 绘制机器人（三角形表示方向）
      ctx.beginPath();
      ctx.moveTo(20, 0);
      ctx.lineTo(-15, -12);
      ctx.lineTo(-15, 12);
      ctx.closePath();
      ctx.fillStyle = '#3b82f6';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.restore();
    }

    // 绘制网格线（可选）
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.lineWidth = 1;
    for (let i = 0; i < width; i += 50) {
      ctx.beginPath();
      ctx.moveTo(i, 0);
      ctx.lineTo(i, height);
      ctx.stroke();
    }
    for (let i = 0; i < height; i += 50) {
      ctx.beginPath();
      ctx.moveTo(0, i);
      ctx.lineTo(width, i);
      ctx.stroke();
    }

    // 绘制坐标信息
    if (mapData) {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
      ctx.fillRect(10, height - 90, 220, 80);
      ctx.fillStyle = '#fff';
      ctx.font = '14px monospace';
      ctx.fillText(`机器人位置:`, 20, height - 65);
      ctx.fillText(`X: ${robotPose.x.toFixed(2)} m`, 20, height - 45);
      ctx.fillText(`Y: ${robotPose.y.toFixed(2)} m`, 20, height - 25);
      ctx.fillText(`θ: ${(robotPose.theta * 180 / Math.PI).toFixed(1)}°`, 20, height - 5);
    }

  }, [mapData, robotPose, scale, offset]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleResize = () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    };

    handleResize();
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      className="w-full h-full"
      style={{ display: 'block' }}
    />
  );
}
