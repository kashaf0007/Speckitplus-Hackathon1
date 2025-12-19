/**
 * TypeScript types for Book RAG Chatbot UI
 *
 * These interfaces define the data structures for chatbot state management,
 * API communication, and UI component props.
 */

/**
 * Represents a single message in the conversation history
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  selectedTextUsed?: string;
  sources?: SourceReference[];
  chunks?: ChunkReference[];
  error?: string;
}

/**
 * Simplified source identifier for display in sources list
 */
export interface SourceReference {
  chunkId: string;
  label: string;
  path?: string;
  anchor?: string;
}

/**
 * Book content chunk with location metadata (from backend AskResponse)
 */
export interface ChunkReference {
  chunk_id: string;
  text: string;
  page: number | null;
  chapter: string | null;
  section: string | null;
}

/**
 * Captured user text selection state
 */
export interface SelectedTextContext {
  text: string;
  startOffset: number;
  endOffset: number;
  capturedAt: Date;
}

/**
 * Request payload for backend /ask endpoint
 */
export interface AskRequest {
  question: string;
  selected_text?: string;
}

/**
 * Response payload from backend /ask endpoint
 */
export interface AskResponse {
  answer: string;
  sources: string[];
  matched_chunks: ChunkReference[];
  grounded: boolean;
  retrieval_quality: string | null;
}

/**
 * Chatbot state managed by React Context
 */
export interface ChatbotState {
  messages: ChatMessage[];
  loading: boolean;
  error: string | null;
  selectedText: SelectedTextContext | null;
}

/**
 * Chatbot context actions
 */
export interface ChatbotContextType extends ChatbotState {
  sendMessage: (question: string) => Promise<void>;
  clearError: () => void;
  setSelectedText: (text: SelectedTextContext | null) => void;
}
