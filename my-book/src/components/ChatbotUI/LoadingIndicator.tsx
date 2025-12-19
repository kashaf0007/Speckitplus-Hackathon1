/**
 * Loading indicator component
 *
 * Displays inline spinner with "Searching..." text during query processing.
 * Non-blocking design allows user to continue interacting with the page.
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Loading indicator props
 */
interface LoadingIndicatorProps {
  /** Optional custom message (default: "Searching book content...") */
  message?: string;
}

/**
 * Loading indicator component
 *
 * Shows spinner animation and loading message below input area
 */
export function LoadingIndicator({ message = 'Searching book content...' }: LoadingIndicatorProps): JSX.Element {
  return (
    <div className={styles.loadingIndicator}>
      <div className={styles.spinner} aria-label="Loading" />
      <span>{message}</span>
    </div>
  );
}
