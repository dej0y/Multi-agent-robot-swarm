import React, { useState, useRef, useEffect } from 'react';
import { MessageSquare, Send, Bug } from 'lucide-react';
import { ChatMessage, ApiResponse, Bot, ObjectData } from '../types/types'; // Make sure to import necessary types
import { sendMessage } from '../api/llamaApi';
import { DebugPanel } from './DebugPanel';
import assignBot  from '../services/assignBot';

export function ChatInterface({ bots, objects }: { bots: Bot[], objects: ObjectData[] }) {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [debugInfo, setDebugInfo] = useState<ApiResponse | null>(null);
  const [showDebug, setShowDebug] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    if (chatContainerRef.current) {
      const { scrollHeight, clientHeight, scrollTop } = chatContainerRef.current;
      const isScrolledNearBottom = scrollHeight - scrollTop - clientHeight < 100;
      if (isScrolledNearBottom) {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
      }
    }
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;
  
    const userMessage: ChatMessage = {
      role: 'user',
      content: input,
      timestamp: new Date().toISOString(),
    };
  
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
  
    try {
      // Send the input to LangChain
      const response = await fetch('http://127.0.0.1:5000/assign', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ user_input: input }),
      });
  
      const data = await response.json();
  
      if (data.success) {
        const botMessage: ChatMessage = {
          role: 'assistant',
          content: data.response,
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, botMessage]);
      } else {
        const errorMessage: ChatMessage = {
          role: 'error',
          content: data.response || 'An error occurred.',
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      }
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        {
          role: 'error',
          content: 'Something went wrong. Please try again.',
          timestamp: new Date().toISOString(),
        },
      ]);
    }
  
    setIsLoading(false);
  };
  
  

  
  return (
    <div className="bg-white rounded-lg shadow-md p-6 flex flex-col h-[600px]">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-2xl font-bold flex items-center gap-2">
          <MessageSquare className="w-6 h-6 text-blue-500" />
          Chat Interface
        </h2>
        <button
          onClick={() => setShowDebug(!showDebug)}
          className="flex items-center gap-1 text-sm text-gray-600 hover:text-gray-900"
        >
          <Bug className="w-4 h-4" />
          {showDebug ? 'Hide' : 'Show'} Debug
        </button>
      </div>

      <div 
        ref={chatContainerRef}
        className="flex-1 overflow-y-auto mb-4 pr-2 scrollbar-thin scrollbar-thumb-gray-300 scrollbar-track-gray-100"
      >
        <div className="space-y-4">
          {messages.map((msg, index) => (
            <div
              key={index}
              className={`p-4 rounded-lg ${
                msg.role === 'user'
                  ? 'bg-blue-50 ml-8'
                  : msg.role === 'error'
                  ? 'bg-red-50'
                  : 'bg-gray-50 mr-8'
              }`}
            >
              <div className="flex items-center gap-2 mb-1">
                <span className="font-medium">
                  {msg.role === 'user' ? 'You' : msg.role === 'error' ? 'Error' : 'Assistant'}
                </span>
                <span className="text-xs text-gray-500">
                  {new Date(msg.timestamp).toLocaleTimeString()}
                </span>
              </div>
              <p className="text-gray-700 break-words">{msg.content}</p>
            </div>
          ))}
          <div ref={messagesEndRef} />
        </div>
      </div>

      <form onSubmit={handleSubmit} className="flex gap-2">
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Type your message..."
          className="flex-1 px-4 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
          disabled={isLoading}
        />
        <button
          type="submit"
          disabled={isLoading}
          className="px-4 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
        >
          <Send className="w-4 h-4" />
          Send
        </button>
      </form>

      {showDebug && debugInfo && (
        <DebugPanel debugInfo={debugInfo} />
      )}
    </div>
  );
}
