import React from 'react';
import { ObjectData } from '../types/types';
import { Package } from 'lucide-react';
import { LocationBadge } from './LocationBadge';

interface ObjectListProps {
  objects: ObjectData[];
  thresholdTime: number;
}

export function ObjectList({ objects, thresholdTime }: ObjectListProps) {
  return (
    <div className="bg-white rounded-lg shadow-md p-6 h-[400px] flex flex-col">
      <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
        <Package className="w-6 h-6 text-blue-500" />
        Objects
      </h2>
      <div className="overflow-y-auto flex-1 pr-2 scrollbar-thin scrollbar-thumb-gray-300 scrollbar-track-gray-100">
        <div className="space-y-4">
          {objects.map((obj) => (
            <div
              key={obj.id}
              className="flex flex-col p-4 bg-gray-50 rounded-lg space-y-2 hover:bg-gray-100 transition-colors"
            >
              <div className="flex items-center justify-between">
                <span className="font-medium text-gray-700">{obj.name}</span>
                <span className="text-sm text-gray-500">ID: {obj.id}</span>
              </div>
              <div className="flex items-center justify-between">
                <LocationBadge x={obj.loc_x} y={obj.loc_y} />
                <span className="text-sm text-gray-500">
                  Timestamp: {obj.timestamp}ms
                </span>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}