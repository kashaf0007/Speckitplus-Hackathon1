/**
 * Chatbot context and provider for global state management (T009, T026)
 *
 * Provides React Context for chatbot state (messages, loading, error, selectedText)
 * and actions (sendMessage, clearError, setSelectedText).
 *
 * Integrates useTextSelection hook to capture user text selection.
 */

import React, { createContext, useContext, useState, useCallback, useEffect, ReactNode, JSX } from 'react';
import { useBackendApi } from './useBackendApi';
import { useTextSelection } from './useTextSelection';
import type {
  ChatMessage,
  ChatbotContextType,
  SelectedTextContext,
  SourceReference,
} from '../types/chatbot';

/**
 * Map chapter titles to their actual doc paths
 * These must match the folder structure in my-book/docs/
 */
const CHAPTER_PATH_MAP: Record<string, string> = {
  // Chapter 1 variations
  'chapter 1': 'chapter-1-introduction',
  'chapter 1: introduction to physical ai': 'chapter-1-introduction',
  'introduction to physical ai': 'chapter-1-introduction',
  'introduction': 'chapter-1-introduction',

  // Chapter 2 variations
  'chapter 2': 'chapter-2-ros2',
  'chapter 2: ros 2 fundamentals': 'chapter-2-ros2',
  'chapter 2: the robotic nervous system (ros 2)': 'chapter-2-ros2',
  'ros 2 fundamentals': 'chapter-2-ros2',
  'ros2 fundamentals': 'chapter-2-ros2',

  // Chapter 3 variations
  'chapter 3': 'chapter-3-simulation',
  'chapter 3: simulation (gazebo & unity)': 'chapter-3-simulation',
  'chapter 3: simulation': 'chapter-3-simulation',
  'simulation (gazebo & unity)': 'chapter-3-simulation',

  // Chapter 4 variations
  'chapter 4': 'chapter-4-isaac',
  'chapter 4: isaac sim': 'chapter-4-isaac',
  'chapter 4: nvidia isaac': 'chapter-4-isaac',
  'isaac sim': 'chapter-4-isaac',
  'nvidia isaac': 'chapter-4-isaac',

  // Chapter 5 variations
  'chapter 5': 'chapter-5-vla',
  'chapter 5: vision-language-action': 'chapter-5-vla',
  'chapter 5: vision-language-action systems': 'chapter-5-vla',
  'vision-language-action': 'chapter-5-vla',
  'vision-language-action systems': 'chapter-5-vla',

  // Chapter 6 variations
  'chapter 6': 'chapter-6-capstone',
  'chapter 6: capstone': 'chapter-6-capstone',
  'chapter 6: capstone project': 'chapter-6-capstone',
  'capstone': 'chapter-6-capstone',
  'capstone project': 'chapter-6-capstone',
};

/**
 * Get the correct doc path for a chapter title
 */
function getChapterPath(chapter: string): string {
  const normalizedChapter = chapter.toLowerCase().trim();

  // Check for exact match first
  if (CHAPTER_PATH_MAP[normalizedChapter]) {
    return CHAPTER_PATH_MAP[normalizedChapter];
  }

  // Check for partial matches
  for (const [key, path] of Object.entries(CHAPTER_PATH_MAP)) {
    if (normalizedChapter.includes(key) || key.includes(normalizedChapter)) {
      return path;
    }
  }

  // Fallback to slugify for unknown chapters
  return chapter.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/(^-|-$)/g, '');
}

/**
 * Slugify text for URL anchor
 */
function slugify(text: string): string {
  return text
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/(^-|-$)/g, '');
}

/**
 * Chat context with default values
 */
const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

/**
 * Maximum number of messages to keep in history (prevent memory bloat)
 */
const MAX_MESSAGES = 100;

/**
 * Provider component props
 */
interface ChatbotProviderProps {
  children: ReactNode;
}

/**
 * Chatbot context provider component
 *
 * Manages conversation state and provides actions for sending messages
 */
export function ChatbotProvider({ children }: ChatbotProviderProps): JSX.Element {
  const [messages, setMessages] = useState<ChatMessage[]>([]);

  // T026: Integrate useTextSelection hook for capturing user text selection
  const {
    selectedText: browserSelectedText,
    setSelectedText: setBrowserSelectedText,
    clearSelection: clearBrowserSelection,
  } = useTextSelection();

  // Local selected text state (can be set manually or from browser selection)
  const [selectedText, setSelectedTextState] = useState<SelectedTextContext | null>(null);

  // Sync browser selection to local state
  useEffect(() => {
    if (browserSelectedText) {
      setSelectedTextState(browserSelectedText);
    }
  }, [browserSelectedText]);

  const { loading, error, askQuestion, clearError } = useBackendApi();

  /**
   * Send a message to the chatbot
   */
  const sendMessage = useCallback(
    async (question: string) => {
      if (!question.trim()) {
        return;
      }

      // Create user message
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'user',
        content: question.trim(),
        timestamp: new Date(),
        selectedTextUsed: selectedText?.text,
      };

      // Add user message to history
      setMessages((prev) => {
        const updated = [...prev, userMessage];
        // Truncate if exceeds max
        return updated.length > MAX_MESSAGES ? updated.slice(-MAX_MESSAGES) : updated;
      });

      // Call backend API
      const response = await askQuestion(question.trim(), selectedText?.text);

      if (response) {
        // Create source references from response
        const sourceRefs: SourceReference[] = response.sources.map((sourceId, index) => {
          const chunk = response.matched_chunks[index];
          let label = sourceId;

          if (chunk) {
            if (chunk.chapter && chunk.section) {
              label = `${chunk.chapter}: ${chunk.section}`;
            } else if (chunk.chapter) {
              label = chunk.chapter;
            } else if (sourceId === 'selected_text') {
              label = 'Selected Text';
            }
          }

          return {
            chunkId: sourceId,
            label,
            path: chunk?.chapter ? `/docs/${getChapterPath(chunk.chapter)}` : undefined,
            anchor: chunk?.section ? slugify(chunk.section) : undefined,
          };
        });

        // Create assistant message
        const assistantMessage: ChatMessage = {
          id: (Date.now() + 1).toString(),
          role: 'assistant',
          content: response.answer,
          timestamp: new Date(),
          sources: sourceRefs,
          chunks: response.matched_chunks,
        };

        setMessages((prev) => {
          const updated = [...prev, assistantMessage];
          return updated.length > MAX_MESSAGES ? updated.slice(-MAX_MESSAGES) : updated;
        });
      } else if (error) {
        // Create error message
        const errorMessage: ChatMessage = {
          id: (Date.now() + 1).toString(),
          role: 'assistant',
          content: '',
          timestamp: new Date(),
          error,
        };

        setMessages((prev) => {
          const updated = [...prev, errorMessage];
          return updated.length > MAX_MESSAGES ? updated.slice(-MAX_MESSAGES) : updated;
        });
      }

      // Clear selected text after sending
      setSelectedTextState(null);
    },
    [selectedText, askQuestion, error]
  );

  /**
   * Set selected text context (T026, T031)
   * Also clears browser selection when setting to null
   */
  const setSelectedText = useCallback((text: SelectedTextContext | null) => {
    setSelectedTextState(text);
    if (!text) {
      // Clear browser selection when manually clearing
      clearBrowserSelection();
    }
  }, [clearBrowserSelection]);

  const contextValue: ChatbotContextType = {
    messages,
    loading,
    error,
    selectedText,
    sendMessage,
    clearError,
    setSelectedText,
  };

  return React.createElement(ChatbotContext.Provider, { value: contextValue }, children);
}

/**
 * Hook to access chatbot context
 *
 * @throws Error if used outside ChatbotProvider
 */
export function useChatbot(): ChatbotContextType {
  const context = useContext(ChatbotContext);

  if (!context) {
    throw new Error('useChatbot must be used within ChatbotProvider');
  }

  return context;
}
