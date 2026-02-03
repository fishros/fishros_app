import { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface RosbridgeContextType {
  ws: WebSocket | null;
  isConnected: boolean;
  connect: (ip: string, port: string) => Promise<void>;
  disconnect: () => void;
  publishMessage: (topic: string, messageType: string, message: any) => void;
  subscribeTopic: (topic: string, messageType: string, callback: (message: any) => void) => string;
  unsubscribeTopic: (subscriptionId: string) => void;
}

const RosbridgeContext = createContext<RosbridgeContextType | undefined>(undefined);

export function RosbridgeProvider({ children }: { children: ReactNode }) {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [subscriptions, setSubscriptions] = useState<Map<string, (message: any) => void>>(new Map());

  const connect = async (ip: string, port: string): Promise<void> => {
    return new Promise((resolve, reject) => {
      try {
        const websocket = new WebSocket(`ws://${ip}:${port}`);
        
        websocket.onopen = () => {
          console.log('Connected to rosbridge');
          setWs(websocket);
          setIsConnected(true);
          resolve();
        };

        websocket.onerror = (error) => {
          console.error('WebSocket error:', error);
          reject(new Error('连接失败'));
        };

        websocket.onclose = () => {
          console.log('Disconnected from rosbridge');
          setIsConnected(false);
          setWs(null);
        };

        websocket.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            
            // 处理订阅消息
            if (data.topic && subscriptions.has(data.topic)) {
              const callback = subscriptions.get(data.topic);
              if (callback) {
                callback(data.msg);
              }
            }
          } catch (error) {
            console.error('Error parsing message:', error);
          }
        };

        // 超时处理
        setTimeout(() => {
          if (websocket.readyState !== WebSocket.OPEN) {
            websocket.close();
            reject(new Error('连接超时'));
          }
        }, 5000);
      } catch (error) {
        reject(error);
      }
    });
  };

  const disconnect = () => {
    if (ws) {
      ws.close();
      setWs(null);
      setIsConnected(false);
    }
  };

  const publishMessage = (topic: string, messageType: string, message: any) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const rosMessage = {
        op: 'publish',
        topic: topic,
        type: messageType,
        msg: message
      };
      ws.send(JSON.stringify(rosMessage));
    }
  };

  const subscribeTopic = (topic: string, messageType: string, callback: (message: any) => void): string => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const subscribeMessage = {
        op: 'subscribe',
        topic: topic,
        type: messageType
      };
      ws.send(JSON.stringify(subscribeMessage));
      
      setSubscriptions(prev => new Map(prev).set(topic, callback));
      return topic;
    }
    return '';
  };

  const unsubscribeTopic = (subscriptionId: string) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const unsubscribeMessage = {
        op: 'unsubscribe',
        topic: subscriptionId
      };
      ws.send(JSON.stringify(unsubscribeMessage));
      
      setSubscriptions(prev => {
        const newMap = new Map(prev);
        newMap.delete(subscriptionId);
        return newMap;
      });
    }
  };

  return (
    <RosbridgeContext.Provider
      value={{
        ws,
        isConnected,
        connect,
        disconnect,
        publishMessage,
        subscribeTopic,
        unsubscribeTopic
      }}
    >
      {children}
    </RosbridgeContext.Provider>
  );
}

export function useRosbridge() {
  const context = useContext(RosbridgeContext);
  if (context === undefined) {
    throw new Error('useRosbridge must be used within a RosbridgeProvider');
  }
  return context;
}
