---
id: 007
title: Fix Sources List Navigation
stage: general
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: feat/book-rag-chatbot-system
user: Kashaf Aman
command: /sp.git.commit_pr
labels: ["git", "commit", "bugfix", "chatbot", "navigation", "sources"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/kashaf0007/Speckitplus-Hackathon1/compare/feat/book-rag-chatbot-system
files:
  - my-book/src/hooks/useChatbot.ts
tests:
  - Manual verification of Sources list navigation
---

## Prompt

User reported clicking chapter links in the Sources section of chatbot responses still showed errors after the previous fix. Sources like "Chapter 1: Introduction to Physical AI" and "Chapter 5: Vision-Language-Action" were still broken.

Command: /sp.git.commit_pr

## Response snapshot

Found that the previous fix only addressed ChunkDisplay.tsx (for "View in book" links), but the Sources list is rendered by SourcesList.tsx which gets its paths from useChatbot.ts.

The path generation in useChatbot.ts line 111 was using simple string replacement:
```typescript
path: `/docs/${chunk.chapter.toLowerCase().replace(/\s+/g, '-')}`
```

This produced broken URLs like `/docs/chapter-1:-introduction-to-physical-ai`.

Fixed by adding the same CHAPTER_PATH_MAP and getChapterPath() function to useChatbot.ts to correctly map chapter titles to their Docusaurus folder paths.

## Outcome

- ✅ Impact: Fixed Sources list navigation in chatbot - both Sources and "View in book" links now work
- 🧪 Tests: Build passes successfully
- 📁 Files: 1 file changed, 80 insertions, 2 deletions
- 🔁 Next prompts: Consider extracting path mapping to shared utility
- 🧠 Reflection: Should have searched for all path generation code in the first fix, not just the visible component

## Evaluation notes (flywheel)

- Failure modes observed: Incomplete fix - missed second location generating paths
- Graders run and results (PASS/FAIL): Build PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Extract CHAPTER_PATH_MAP to shared module to avoid duplication
