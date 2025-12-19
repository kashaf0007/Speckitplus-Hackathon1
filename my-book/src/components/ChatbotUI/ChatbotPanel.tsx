/**
 * Chatbot panel component (T014)
 *
 * Modal container for the chatbot interface.
 * Contains message list, query input, and error display.
 * Dismissible with close button or Escape key.
 */

import React, { useEffect, useCallback } from 'react';
import styles from './styles.module.css';
import { MessageList } from './MessageList';
import { QueryInput } from './QueryInput';
import { ErrorMessage } from './ErrorMessage';
import { useChatbot } from '../../hooks/useChatbot';

/**
 * Chatbot panel props
 */
interface ChatbotPanelProps {
  /** Whether the panel is visible */
  isOpen: boolean;
  /** Callback to close the panel */
  onClose: () => void;
}

/**
 * Chatbot panel component
 *
 * Main container for chatbot UI with:
 * - Header with title and close button
 * - Message list for conversation history
 * - Query input for user questions
 * - Error display for API errors
 */
export function ChatbotPanel({ isOpen, onClose }: ChatbotPanelProps): JSX.Element {
  const {
    messages,
    loading,
    error,
    selectedText,
    sendMessage,
    clearError,
    setSelectedText,
  } = useChatbot();

  /**
   * Handle Escape key to close panel
   */
  const handleKeyDown = useCallback(
    (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    },
    [isOpen, onClose]
  );

  // Add/remove keyboard listener
  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  if (!isOpen) {
    return null;
  }

  /**
   * Handle sending a message
   */
  const handleSend = async (question: string) => {
    await sendMessage(question);
  };

  /**
   * Clear selected text
   */
  const handleClearSelection = () => {
    setSelectedText(null);
  };

  return (
    <div className={styles.panel} role="dialog" aria-label="Book chatbot">
      {/* Header */}
      <div className={styles.panelHeader}>
        <h3 className={styles.panelTitle}>📚 Book Assistant</h3>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chatbot"
          type="button"
        >
          ×
        </button>
      </div>

      {/* Error display */}
      {error && (
        <div style={{ padding: '0 16px' }}>
          <ErrorMessage message={error} onDismiss={clearError} />
        </div>
      )}

      {/* Message list */}
      <MessageList messages={messages} loading={loading} />

      {/* Query input */}
      <QueryInput
        onSend={handleSend}
        loading={loading}
        selectedText={selectedText}
        onClearSelection={handleClearSelection}
      />
    </div>
  );
}
