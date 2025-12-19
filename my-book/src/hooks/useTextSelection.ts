/**
 * Text selection hook (T025)
 *
 * Captures user text selection using window.getSelection() API.
 * Listens for mouseup events to detect selection changes.
 * Trims whitespace and truncates to 2000 characters max.
 */

import { useState, useEffect, useCallback } from 'react';
import type { SelectedTextContext } from '../types/chatbot';

/**
 * Maximum characters to capture from selection
 */
const MAX_SELECTION_LENGTH = 2000;

/**
 * Hook return type
 */
interface UseTextSelectionReturn {
  /** Current selected text (null if none) */
  selectedText: SelectedTextContext | null;
  /** Manually set selected text */
  setSelectedText: (text: SelectedTextContext | null) => void;
  /** Clear current selection */
  clearSelection: () => void;
}

/**
 * Custom hook for capturing text selection
 *
 * @returns Object with selectedText state and control functions
 */
export function useTextSelection(): UseTextSelectionReturn {
  const [selectedText, setSelectedText] = useState<SelectedTextContext | null>(null);

  /**
   * Handle mouseup event to capture selection
   */
  const handleMouseUp = useCallback(() => {
    // Get current selection
    const selection = window.getSelection();

    if (!selection || selection.isCollapsed) {
      // No selection or collapsed (cursor position only)
      return;
    }

    // Get selected text
    const text = selection.toString().trim();

    if (!text) {
      return;
    }

    // Skip if selection is inside chatbot panel
    const anchorNode = selection.anchorNode;
    if (anchorNode) {
      const element = anchorNode.parentElement;
      if (element?.closest('[role="dialog"]')) {
        // Selection is inside chatbot dialog, ignore
        return;
      }
    }

    // Truncate if too long
    const truncatedText = text.length > MAX_SELECTION_LENGTH
      ? text.substring(0, MAX_SELECTION_LENGTH)
      : text;

    // Get range for offset positions
    const range = selection.getRangeAt(0);

    // Create selection context
    const context: SelectedTextContext = {
      text: truncatedText,
      startOffset: range.startOffset,
      endOffset: range.endOffset,
      capturedAt: new Date(),
    };

    setSelectedText(context);
  }, []);

  /**
   * Clear current selection
   */
  const clearSelection = useCallback(() => {
    setSelectedText(null);
    // Also clear browser selection
    const selection = window.getSelection();
    if (selection) {
      selection.removeAllRanges();
    }
  }, []);

  // Add mouseup listener on mount
  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [handleMouseUp]);

  return {
    selectedText,
    setSelectedText,
    clearSelection,
  };
}
