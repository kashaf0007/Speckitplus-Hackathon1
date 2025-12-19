/**
 * Answer display component (T017)
 *
 * Renders assistant message with answer text.
 * Handles refusal messages with special styling.
 * Shows indicator when answer is based on selected text.
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Answer display props
 */
interface AnswerDisplayProps {
  /** Answer text content */
  answer: string;
  /** Whether the answer is grounded in book content */
  grounded: boolean;
  /** Whether the answer is based on selected text */
  isFromSelectedText?: boolean;
}

/**
 * Check if answer is a refusal message
 */
function isRefusalMessage(answer: string): boolean {
  const refusalIndicators = [
    'not available in the book',
    'couldn\'t find',
    'could not find',
    'no relevant information',
    'not covered in the book',
    'unable to find',
  ];

  const lowerAnswer = answer.toLowerCase();
  return refusalIndicators.some((indicator) => lowerAnswer.includes(indicator));
}

/**
 * Answer display component
 *
 * Shows answer text with optional indicators for:
 * - Refusal messages (when content not in book)
 * - Selected text context (when answer uses user selection)
 */
export function AnswerDisplay({
  answer,
  grounded,
  isFromSelectedText,
}: AnswerDisplayProps): JSX.Element {
  const isRefusal = isRefusalMessage(answer) || !grounded;

  return (
    <div className={styles.answerDisplay}>
      {/* Selected text indicator */}
      {isFromSelectedText && !isRefusal && (
        <div className={styles.answerBadge}>
          📎 Answer based on selected text
        </div>
      )}

      {/* Answer content */}
      <div className={isRefusal ? styles.refusalAnswer : undefined}>
        {isRefusal && <span className={styles.refusalIcon}>ℹ️ </span>}
        {answer}
      </div>

      {/* Suggestions for refusal */}
      {isRefusal && (
        <div className={styles.refusalSuggestions}>
          <p>Try:</p>
          <ul>
            <li>Using different keywords</li>
            <li>Asking about a specific chapter or topic</li>
            <li>Selecting relevant text and asking about it</li>
          </ul>
        </div>
      )}
    </div>
  );
}
