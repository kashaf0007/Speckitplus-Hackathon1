/**
 * React hook for backend API communication
 *
 * Wraps chatbotApi service with error handling and loading state management.
 */

import { useState, useCallback } from 'react';
import { askQuestion, type ClassifiedError } from '../services/chatbotApi';
import type { AskResponse } from '../types/chatbot';

/**
 * Hook return type
 */
interface UseBackendApiReturn {
  loading: boolean;
  error: string | null;
  askQuestion: (question: string, selectedText?: string) => Promise<AskResponse | null>;
  clearError: () => void;
}

/**
 * Custom hook for backend API calls with error handling
 *
 * @returns Object with loading state, error state, askQuestion function, and clearError function
 */
export function useBackendApi(): UseBackendApiReturn {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Ask a question to the backend
   */
  const ask = useCallback(
    async (question: string, selectedText?: string): Promise<AskResponse | null> => {
      setLoading(true);
      setError(null);

      try {
        const response = await askQuestion(question, selectedText);
        setLoading(false);
        return response;
      } catch (err) {
        setLoading(false);

        // Handle classified errors
        if ((err as ClassifiedError).message) {
          setError((err as ClassifiedError).message);
        } else {
          // Fallback for unexpected errors
          setError('An unexpected error occurred. Please try again.');
        }

        return null;
      }
    },
    []
  );

  /**
   * Clear error state
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    loading,
    error,
    askQuestion: ask,
    clearError,
  };
}
