export interface Location {
  loc_x: number;
  loc_y: number;
}

export interface Bot extends Location {
  name: string;
  status: number;
}

export interface ObjectData extends Location {
  name: string;
  id: number;
  timestamp: number;
}

export interface DashboardData {
  map: string;
  Threshold_time: number;
  objects: ObjectData[];
  bots: Bot[];
}

export interface ChatMessage {
  role: 'user' | 'assistant' | 'error';
  content: string;
  timestamp: string;
}

export interface ApiResponse {
  success: boolean;
  data?: any;
  error?: string;
  debug: {
    status?: number;
    headers?: Record<string, string>;
    timestamp: string;
    error?: string;
  };
}