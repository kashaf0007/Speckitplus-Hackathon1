/**
 * Chatbot API service for communicating with FastAPI backend
 *
 * Provides askQuestion function with:
 * - Fetch API for HTTP requests
 * - 5-second AbortController timeout
 * - Three-tier error classification (network, 400, 500)
 */

import type { AskRequest, AskResponse } from '../types/chatbot';

const API_BASE_URL = 'http://localhost:8000';
const REQUEST_TIMEOUT = 60000; // 60 seconds - increased for slow API calls

/**
 * Error types for classification
 */
export enum ErrorType {
  NETWORK = 'network',
  TIMEOUT = 'timeout',
  BAD_REQUEST = 'bad_request',
  SERVER_ERROR = 'server_error',
  UNKNOWN = 'unknown',
}

/**
 * Classified error with user-friendly message
 */
export interface ClassifiedError {
  type: ErrorType;
  message: string;
  originalError?: Error;
}

/**
 * Classify error for user-friendly display
 */
function classifyError(error: any, response?: Response): ClassifiedError {
  // Timeout error
  if (error.name === 'AbortError') {
    return {
      type: ErrorType.TIMEOUT,
      message: 'Request timed out. The book search took too long. Please try again.',
      originalError: error,
    };
  }

  // Network error (offline, DNS failure, etc.)
  if (error.message === 'Failed to fetch' || error.message.includes('fetch')) {
    return {
      type: ErrorType.NETWORK,
      message: 'Unable to connect. Check your internet connection.',
      originalError: error,
    };
  }

  // HTTP error responses
  if (response) {
    if (response.status === 400) {
      return {
        type: ErrorType.BAD_REQUEST,
        message: error.message || 'Invalid request. Please check your question.',
        originalError: error,
      };
    }

    if (response.status === 429) {
      return {
        type: ErrorType.SERVER_ERROR,
        message: 'API rate limit reached. Please wait a moment and try again.',
        originalError: error,
      };
    }

    if (response.status >= 500) {
      return {
        type: ErrorType.SERVER_ERROR,
        message: 'Something went wrong on our end. Please try again later.',
        originalError: error,
      };
    }
  }

  // Unknown error
  return {
    type: ErrorType.UNKNOWN,
    message: 'An unexpected error occurred. Please try again.',
    originalError: error,
  };
}

/**
 * Ask a question to the RAG chatbot backend
 *
 * @param question - User's natural language question
 * @param selectedText - Optional user-selected text for context override
 * @returns Promise resolving to AskResponse with answer, sources, and chunks
 * @throws ClassifiedError with user-friendly error message
 */
export async function askQuestion(
  question: string,
  selectedText?: string
): Promise<AskResponse> {
  // Create abort controller for timeout
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT);

  try {
    // Prepare request body
    const requestBody: AskRequest = {
      question: question.trim(),
    };

    if (selectedText && selectedText.trim()) {
      requestBody.selected_text = selectedText.trim();
    }

    // Make request to backend
    const response = await fetch(`${API_BASE_URL}/ask`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
      signal: controller.signal,
    });

    // Clear timeout on successful response
    clearTimeout(timeoutId);

    // Handle non-OK responses
    if (!response.ok) {
      let errorMessage = 'Request failed';

      try {
        const errorData = await response.json();
        errorMessage = errorData.detail || errorMessage;
      } catch {
        // If error response is not JSON, use default message
      }

      const error = new Error(errorMessage);
      throw classifyError(error, response);
    }

    // Parse successful response
    const data: AskResponse = await response.json();
    return data;
  } catch (error) {
    // Clear timeout on error
    clearTimeout(timeoutId);

    // If error is already classified, rethrow it
    if ((error as any).type) {
      throw error;
    }

    // Classify and throw error
    throw classifyError(error);
  }
}
