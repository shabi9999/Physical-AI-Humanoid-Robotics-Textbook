/**
 * RAG Chatbot Component
 *
 * Interactive chatbot powered by the backend RAG system.
 * This is a placeholder component to be fully implemented with:
 * - Message UI
 * - API integration with /api/query endpoint
 * - Loading states
 * - Error handling
 * - Citation display
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './RAGChatbot.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter: string;
    url: string;
    excerpt: string;
  }>;
}

interface RAGChatbotProps {
  apiUrl?: string;
}

export const RAGChatbot: React.FC<RAGChatbotProps> = ({
  apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000',
}) => {
  const [messages, setMessages] = useState<Message[]>([
    {
      role: 'assistant',
      content: 'Hi! I\'m the ROS 2 course assistant. Ask me anything about ROS 2 fundamentals, agent bridging, or URDF modeling!',
    },
  ]);

  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!input.trim()) return;

    // Add user message
    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Call RAG API
      const response = await fetch(`${apiUrl}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: input,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant message with sources
      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Failed to get response:', error);

      const errorMessage: Message = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again later or check if the backend is running.',
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbot}>
      <div className={styles.header}>
        <h3>ROS 2 Course Assistant</h3>
        <p>Powered by RAG</p>
      </div>

      <div className={styles.messages}>
        {messages.map((msg, idx) => (
          <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
            <div className={styles.content}>{msg.content}</div>

            {msg.sources && msg.sources.length > 0 && (
              <div className={styles.sources}>
                <strong>Sources:</strong>
                <ul>
                  {msg.sources.map((source, sidx) => (
                    <li key={sidx}>
                      <a href={source.url} target="_blank" rel="noopener noreferrer">
                        {source.chapter}
                      </a>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}

        {loading && (
          <div className={`${styles.message} ${styles.assistant}`}>
            <div className={styles.loading}>Thinking...</div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form className={styles.input} onSubmit={handleSubmit}>
        <input
          type="text"
          value={input}
          onChange={e => setInput(e.target.value)}
          placeholder="Ask a question about the course..."
          disabled={loading}
        />
        <button type="submit" disabled={loading || !input.trim()}>
          Send
        </button>
      </form>
    </div>
  );
};

export default RAGChatbot;
