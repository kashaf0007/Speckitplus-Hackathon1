/**
 * Message list component (T015)
 *
 * Renders conversation history with scrollable area.
 * Displays user and assistant messages in chat bubble format.
 * Auto-scrolls to latest message.
 */

import React, { useRef, useEffect } from 'react';
import styles from './styles.module.css';
import type { ChatMessage } from '../../types/chatbot';
import { SourcesList } from './SourcesList';
import { ChunkDisplay } from './ChunkDisplay';

/**
 * Message list props
 */
interface MessageListProps {
  /** Array of chat messages to display */
  messages: ChatMessage[];
  /** Whether a message is currently loading */
  loading?: boolean;
}

/**
 * Format timestamp for display
 */
function formatTime(date: Date): string {
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

/**
 * Message list component
 *
 * Scrollable container for conversation history.
 * Shows empty state when no messages exist.
 */
export function MessageList({ messages, loading }: MessageListProps): JSX.Element {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, loading]);

  // Empty state
  if (messages.length === 0 && !loading) {
    return (
      <div className={styles.messageList}>
        <div className={styles.emptyState}>
          Ask a question about the book to get started
        </div>
      </div>
    );
  }

  return (
    <div className={styles.messageList}>
      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.message} ${
            message.role === 'user' ? styles.messageUser : styles.messageAssistant
          }`}
        >
          <div
            className={`${styles.messageBubble} ${
              message.role === 'user'
                ? styles.messageBubbleUser
                : styles.messageBubbleAssistant
            }`}
          >
            {/* Show error state if present */}
            {message.error ? (
              <span style={{ color: '#c33' }}>{message.error}</span>
            ) : (
              message.content
            )}
          </div>

          {/* Show selected text indicator for user messages */}
          {message.role === 'user' && message.selectedTextUsed && (
            <div className={styles.selectedTextBadge} style={{ maxWidth: '80%', marginTop: '4px' }}>
              <span>📎 With selected text</span>
            </div>
          )}

          {/* Show sources and chunks for assistant messages */}
          {message.role === 'assistant' && !message.error && (
            <>
              {message.sources && message.sources.length > 0 && (
                <SourcesList sources={message.sources} />
              )}
              {message.chunks && message.chunks.length > 0 && (
                <ChunkDisplay chunks={message.chunks} />
              )}
            </>
          )}

          <div className={styles.messageTimestamp}>
            {formatTime(new Date(message.timestamp))}
          </div>
        </div>
      ))}

      {/* Loading indicator at bottom */}
      {loading && (
        <div className={styles.message + ' ' + styles.messageAssistant}>
          <div className={styles.messageBubbleAssistant + ' ' + styles.messageBubble}>
            <div className={styles.loadingIndicator}>
              <div className={styles.spinner} />
              <span>Thinking...</span>
            </div>
          </div>
        </div>
      )}

      {/* Scroll anchor */}
      <div ref={messagesEndRef} />
    </div>
  );
}
