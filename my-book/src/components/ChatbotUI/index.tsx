/**
 * ChatbotUI main component (T020)
 *
 * Root component that integrates toggle button and panel.
 * Manages panel open/close state with persistence.
 * Renders on all book pages via Root.tsx wrapper.
 */

import React, { useState, useEffect } from 'react';
import { ChatbotToggle } from './ChatbotToggle';
import { ChatbotPanel } from './ChatbotPanel';

/**
 * Session storage key for panel state persistence
 */
const PANEL_STATE_KEY = 'chatbot-panel-open';

/**
 * ChatbotUI component
 *
 * Main entry point for chatbot interface.
 * Shows floating toggle button when closed, panel when open.
 * Persists panel state across page navigation.
 */
export function ChatbotUI(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  // Restore panel state from session storage on mount
  useEffect(() => {
    try {
      const savedState = sessionStorage.getItem(PANEL_STATE_KEY);
      if (savedState === 'true') {
        setIsOpen(true);
      }
    } catch {
      // Session storage not available
    }
  }, []);

  // Persist panel state to session storage
  useEffect(() => {
    try {
      sessionStorage.setItem(PANEL_STATE_KEY, String(isOpen));
    } catch {
      // Session storage not available
    }
  }, [isOpen]);

  /**
   * Toggle panel visibility
   */
  const handleToggle = () => {
    setIsOpen((prev) => !prev);
  };

  /**
   * Close panel
   */
  const handleClose = () => {
    setIsOpen(false);
  };

  return (
    <>
      <ChatbotToggle isOpen={isOpen} onToggle={handleToggle} />
      <ChatbotPanel isOpen={isOpen} onClose={handleClose} />
    </>
  );
}

// Export all components for direct usage
export { ChatbotToggle } from './ChatbotToggle';
export { ChatbotPanel } from './ChatbotPanel';
export { MessageList } from './MessageList';
export { QueryInput } from './QueryInput';
export { AnswerDisplay } from './AnswerDisplay';
export { SourcesList } from './SourcesList';
export { ChunkDisplay } from './ChunkDisplay';
export { LoadingIndicator } from './LoadingIndicator';
export { ErrorMessage } from './ErrorMessage';
