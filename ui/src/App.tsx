import React from 'react';
import { BotList } from './components/BotList';
import { ObjectList } from './components/ObjectList';
import { MapVisualization } from './components/MapVisualization';
import { ChatInterface } from './components/ChatInterface';
import { useDataRefresh } from './hooks/useDataRefresh';
import { Bot } from 'lucide-react';
import { LoadingSpinner } from './components/LoadingSpinner';
import { ErrorDisplay } from './components/ErrorDisplay';

function App() {
  const { data, error, isLoading } = useDataRefresh();

  if (isLoading) {
    return <LoadingSpinner />;
  }

  if (error) {
    return <ErrorDisplay error={error} />;
  }

  if (!data) {
    return null;
  }

  return (
    <div className="min-h-screen bg-gray-100 p-8">
      <div className="max-w-7xl mx-auto">
        <header className="mb-8">
          <h1 className="text-3xl font-bold text-gray-900 flex items-center gap-2">
            <Bot className="w-8 h-8 text-blue-500" />
            Bot Management Dashboard
          </h1>
        </header>

        <div className="grid grid-cols-1 lg:grid-cols-12 gap-8">
          <div className="lg:col-span-7 space-y-8">
            <MapVisualization 
              bots={data.bots}
              objects={data.objects}
              map={data.map}
            />
            <ChatInterface />
          </div>
          <div className="lg:col-span-5 space-y-8">
            <BotList bots={data.bots} />
            <ObjectList 
              objects={data.objects}
              thresholdTime={data.Threshold_time}
            />
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;