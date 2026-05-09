import React from 'react';
import { Bot } from '../types/types';
import { Activity } from 'lucide-react';
import { LocationBadge } from './LocationBadge';

interface BotListProps {
  bots: Bot[];
}

export function BotList({ bots }: BotListProps) {
  const getBotStatusText = (status: number) => status === 1 ? 'active' : 'inactive';
  
  return (
    <div className="bg-white rounded-lg shadow-md p-6 h-[400px] flex flex-col">
      <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
        <Activity className="w-6 h-6 text-blue-500" />
        Bot Status
      </h2>
      <div className="overflow-y-auto flex-1 pr-2 scrollbar-thin scrollbar-thumb-gray-300 scrollbar-track-gray-100">
        <div className="space-y-4">
          {bots.map((bot, index) => (
            <div
              key={index}
              className="flex flex-col p-4 bg-gray-50 rounded-lg space-y-2 hover:bg-gray-100 transition-colors"
            >
              <div className="flex items-center justify-between">
                <span className="font-medium text-gray-700">{bot.name}</span>
                <span
                  className={`px-3 py-1 rounded-full text-sm font-medium ${
                    bot.status === 1
                      ? 'bg-green-100 text-green-800'
                      : 'bg-red-100 text-red-800'
                  }`}
                >
                  {getBotStatusText(bot.status)}
                </span>
              </div>
              <LocationBadge x={bot.loc_x} y={bot.loc_y} />
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}