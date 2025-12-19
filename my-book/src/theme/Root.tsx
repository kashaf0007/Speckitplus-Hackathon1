/**
 * Root wrapper component for Docusaurus (T010, T020)
 *
 * This component wraps the entire application and provides global context
 * for the chatbot UI. It ensures the chatbot is available on all pages.
 *
 * Swizzled from @docusaurus/theme-classic
 */

import React, { JSX } from 'react';
import { ChatbotProvider } from '../hooks/useChatbot';
import { ChatbotUI } from '../components/ChatbotUI';

export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <ChatbotProvider>
      {children}
      <ChatbotUI />
    </ChatbotProvider>
  );
}
