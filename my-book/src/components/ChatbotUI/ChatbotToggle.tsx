/**
 * Chatbot toggle button component (T013)
 *
 * Floating 💬 icon button in bottom-right corner.
 * Opens/closes the chatbot panel on click.
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Toggle button props
 */
interface ChatbotToggleProps {
  /** Whether the panel is currently open */
  isOpen: boolean;
  /** Callback to toggle panel visibility */
  onToggle: () => void;
}

/**
 * Floating toggle button component
 *
 * Renders as circular button with chat icon.
 * Supports keyboard accessibility (Enter/Space to activate).
 */
export function ChatbotToggle({ isOpen, onToggle }: ChatbotToggleProps): JSX.Element {
  // Hide toggle when panel is open
  if (isOpen) {
    return null;
  }

  return (
    <button
      className={styles.toggle}
      onClick={onToggle}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          onToggle();
        }
      }}
      aria-label="Open chatbot"
      aria-expanded={isOpen}
      type="button"
    >
      💬
    </button>
  );
}
