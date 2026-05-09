import React from 'react';
import { Bot, ObjectData } from '../types/types';
import { Bot as BotIcon, Package } from 'lucide-react';

interface MapVisualizationProps {
  bots: Bot[];
  objects: ObjectData[];
  gridSize?: number;
  map?: string;
}

export function MapVisualization({ bots, objects, gridSize = 10, map}: MapVisualizationProps) {
  const cellSize = 50; // pixels
  const gridSizePx = cellSize * gridSize;

  if (!gridSize || gridSize <= 0) {
    console.error("Invalid grid size:", gridSize);
    return <div>Error: Invalid grid size</div>;
  }

  // Create grid cells
  const gridCells = Array.from({ length: gridSize }, (_, y) =>
    Array.from({ length: gridSize }, (_, x) => ({ x, y }))
  );

  return (
    <div className="bg-white rounded-lg shadow-md p-6">
      <h2 className="text-2xl font-bold mb-4">Map View</h2>
      <div className="overflow-auto">
        <div
          className="relative bg-gray-50 border border-gray-200"
          style={{
            width: gridSizePx,
            height: gridSizePx,
            backgroundImage: map ? `url(${map})` : 'none',
            backgroundSize: 'cover',
            backgroundPosition: 'center',
          }}
        >
          {/* Grid lines */}
          {!map && gridCells.map((row, y) =>
            row.map((cell, x) => (
              <div
                key={`${x}-${y}`}
                className="absolute border border-gray-200"
                style={{
                  left: x * cellSize,
                  top: y * cellSize,
                  width: cellSize,
                  height: cellSize,
                }}
              >
                <span className="text-xs text-gray-400 absolute bottom-0 right-1">
                  {x},{y}
                </span>
              </div>
            ))
          )}

          {/* Bots */}
          {bots.map((bot, index) => (
            <div
              key={`bot-${index}`}
              className="absolute transform -translate-x-1/2 -translate-y-1/2"
              style={{
                left: bot.loc_x * cellSize,
                top: bot.loc_y * cellSize,
              }}
            >
              <div className={`p-1 rounded-full ${
                bot.status === 1 ? 'bg-green-100' : 'bg-red-100'
              }`}>
                <BotIcon className="w-6 h-6 text-blue-500" />
              </div>
              <span className="text-xs font-medium absolute top-full left-1/2 transform -translate-x-1/2">
                {bot.name}
              </span>
            </div>
          ))}

          {/* Objects */}
          {objects.map((obj) => (
            <div
              key={`object-${obj.id}`}
              className="absolute transform -translate-x-1/2 -translate-y-1/2"
              style={{
                left: obj.loc_x * cellSize,
                top: obj.loc_y * cellSize,
              }}
            >
              <div className="p-1 rounded-full bg-yellow-100">
                <Package className="w-6 h-6 text-yellow-600" />
              </div>
              <span className="text-xs font-medium absolute top-full left-1/2 transform -translate-x-1/2">
                {obj.name}
              </span>
            </div>
          ))}
        </div>
      </div>
      <div className="mt-4 flex gap-4">
        <div className="flex items-center gap-2">
          <div className="p-1 rounded-full bg-blue-100">
            <BotIcon className="w-4 h-4 text-blue-500" />
          </div>
          <span className="text-sm">Bots</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="p-1 rounded-full bg-yellow-100">
            <Package className="w-4 h-4 text-yellow-600" />
          </div>
          <span className="text-sm">Objects</span>
        </div>
      </div>
    </div>
  );
}