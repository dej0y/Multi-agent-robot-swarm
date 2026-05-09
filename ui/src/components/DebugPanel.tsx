import React from 'react';
import { ApiResponse } from '../types/types';

interface DebugPanelProps {
  debugInfo: ApiResponse;
}

export function DebugPanel({ debugInfo }: DebugPanelProps) {
  return (
    <div className="mt-4 p-4 bg-gray-50 rounded-lg">
      <h3 className="text-sm font-medium text-gray-700 mb-2">Debug Information</h3>
      <pre className="text-xs text-gray-600 overflow-x-auto">
        {JSON.stringify(debugInfo, null, 2)}
      </pre>
    </div>
  );
}