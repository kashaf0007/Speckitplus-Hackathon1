/**
 * Error message component
 *
 * Displays user-friendly error messages with dismiss button.
 * Handles network errors, backend errors, and empty results gracefully.
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Error message props
 */
interface ErrorMessageProps {
  /** Error message to display */
  message: string;
  /** Callback when error is dismissed */
  onDismiss?: () => void;
}

/**
 * Error message component
 *
 * Shows error with left border, background color, and dismiss button
 */
export function ErrorMessage({ message, onDismiss }: ErrorMessageProps): JSX.Element {
  return (
    <div className={styles.errorMessage}>
      <span>{message}</span>
      {onDismiss && (
        <button
          className={styles.dismissError}
          onClick={onDismiss}
          aria-label="Dismiss error"
          type="button"
        >
          ×
        </button>
      )}
    </div>
  );
}
