import React from 'react';
import { MapPin } from 'lucide-react';

interface LocationBadgeProps {
  x: number;
  y: number;
}

export function LocationBadge({ x, y }: LocationBadgeProps) {
  return (
    <span className="flex items-center gap-1 text-sm text-gray-600">
      <MapPin className="w-4 h-4" />
      <span>({x}, {y})</span>
    </span>
  );
}