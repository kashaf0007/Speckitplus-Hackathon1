/**
 * Query input component (T016)
 *
 * Textarea input with Send button for user queries.
 * Includes empty query validation and selected text badge display.
 * Supports keyboard shortcuts: Enter to send, Shift+Enter for newline.
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import type { SelectedTextContext } from '../../types/chatbot';
import { LoadingIndicator } from './LoadingIndicator';

/**
 * Query input props
 */
interface QueryInputProps {
  /** Callback when user sends a message */
  onSend: (question: string) => void;
  /** Whether a query is currently loading */
  loading: boolean;
  /** Currently selected text (if any) */
  selectedText: SelectedTextContext | null;
  /** Callback to clear selected text */
  onClearSelection: () => void;
}

/**
 * Query input component
 *
 * Provides input field with send button.
 * Shows selected text badge when text is selected.
 * Disables send when input is empty or loading.
 */
export function QueryInput({
  onSend,
  loading,
  selectedText,
  onClearSelection,
}: QueryInputProps): JSX.Element {
  const [input, setInput] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Focus textarea on mount
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.focus();
    }
  }, []);

  /**
   * Handle send button click
   */
  const handleSend = () => {
    const trimmed = input.trim();
    if (trimmed && !loading) {
      onSend(trimmed);
      setInput('');
    }
  };

  /**
   * Handle keyboard shortcuts
   */
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    // Enter to send (without Shift)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  /**
   * Truncate text for preview
   */
  const truncateText = (text: string, maxLength: number = 50): string => {
    if (text.length <= maxLength) return text;
    return text.substring(0, maxLength) + '...';
  };

  const canSend = input.trim().length > 0 && !loading;

  return (
    <div className={styles.inputContainer}>
      {/* Selected text badge */}
      {selectedText && (
        <div className={styles.selectedTextBadge}>
          <span>📎 Selected: "{truncateText(selectedText.text)}"</span>
          <button
            className={styles.clearSelection}
            onClick={onClearSelection}
            aria-label="Clear selection"
            type="button"
          >
            ×
          </button>
        </div>
      )}

      {/* Input row */}
      <div className={styles.inputRow}>
        <textarea
          ref={textareaRef}
          className={styles.textarea}
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book..."
          disabled={loading}
          rows={1}
          aria-label="Question input"
        />
        <button
          className={styles.sendButton}
          onClick={handleSend}
          disabled={!canSend}
          aria-label="Send question"
          type="button"
        >
          Send
        </button>
      </div>

      {/* Loading indicator */}
      {loading && <LoadingIndicator />}
    </div>
  );
}
