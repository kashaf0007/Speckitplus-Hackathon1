/**
 * React hook for backend API communication
 *
 * Wraps chatbotApi service with error handling, loading state management,
 * and debouncing to prevent rapid requests.
 */

import { useState, useCallback, useRef } from 'react';
import { askQuestion, type ClassifiedError } from '../services/chatbotApi';
import type { AskResponse } from '../types/chatbot';

/**
 * Minimum time between requests (in ms) to prevent rapid fire
 */
const MIN_REQUEST_INTERVAL = 2000;

/**
 * Hook return type
 */
interface UseBackendApiReturn {
  loading: boolean;
  error: string | null;
  askQuestion: (question: string, selectedText?: string) => Promise<AskResponse | null>;
  clearError: () => void;
  isThrottled: boolean;
}

/**
 * Custom hook for backend API calls with error handling and throttling
 *
 * @returns Object with loading state, error state, askQuestion function, clearError function, and throttle state
 */
export function useBackendApi(): UseBackendApiReturn {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isThrottled, setIsThrottled] = useState(false);

  // Track last request time for throttling
  const lastRequestTimeRef = useRef<number>(0);
  const throttleTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  /**
   * Ask a question to the backend with throttling
   */
  const ask = useCallback(
    async (question: string, selectedText?: string): Promise<AskResponse | null> => {
      const now = Date.now();
      const timeSinceLastRequest = now - lastRequestTimeRef.current;

      // Check if we're within the throttle window
      if (timeSinceLastRequest < MIN_REQUEST_INTERVAL && lastRequestTimeRef.current > 0) {
        const waitTime = MIN_REQUEST_INTERVAL - timeSinceLastRequest;
        setIsThrottled(true);
        setError(`Please wait ${Math.ceil(waitTime / 1000)} seconds before asking another question.`);

        // Clear throttle state after wait time
        if (throttleTimeoutRef.current) {
          clearTimeout(throttleTimeoutRef.current);
        }
        throttleTimeoutRef.current = setTimeout(() => {
          setIsThrottled(false);
          setError(null);
        }, waitTime);

        return null;
      }

      setLoading(true);
      setError(null);
      setIsThrottled(false);
      lastRequestTimeRef.current = now;

      try {
        const response = await askQuestion(question, selectedText);
        setLoading(false);
        return response;
      } catch (err) {
        setLoading(false);

        // Handle classified errors
        if ((err as ClassifiedError).message) {
          const errorMessage = (err as ClassifiedError).message;
          setError(errorMessage);

          // If rate limited, set throttle state
          if (errorMessage.toLowerCase().includes('rate limit') ||
              errorMessage.toLowerCase().includes('wait')) {
            setIsThrottled(true);
            // Auto-clear throttle after 10 seconds
            if (throttleTimeoutRef.current) {
              clearTimeout(throttleTimeoutRef.current);
            }
            throttleTimeoutRef.current = setTimeout(() => {
              setIsThrottled(false);
            }, 10000);
          }
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
    setIsThrottled(false);
  }, []);

  return {
    loading,
    error,
    askQuestion: ask,
    clearError,
    isThrottled,
  };
}
