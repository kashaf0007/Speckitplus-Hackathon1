---
id: 006
title: Git Commit PR View In Book Fix
stage: general
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: feat/book-rag-chatbot-system
user: Kashaf Aman
command: /sp.git.commit_pr
labels: ["git", "commit", "pr", "bugfix", "chatbot", "navigation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: https://github.com/kashaf0007/Speckitplus-Hackathon1/compare/feat/book-rag-chatbot-system
files:
  - my-book/src/components/ChatbotUI/ChunkDisplay.tsx
  - Backend/agent_rag.py
  - Backend/api.py
  - Backend/rate_limiter.py
  - Backend/retrieval_validation/answer_detector.py
  - Backend/retrieval_validation/validator.py
  - my-book/src/components/ChatbotUI/ChatbotToggle.tsx
  - my-book/src/services/chatbotApi.ts
tests:
  - Manual verification of View in book navigation
---

## Prompt

User reported "page not found" errors when clicking "View in book" links in chatbot source references. The issue occurred when clicking chapter links like "Chapter 2: ROS 2 Fundamentals" or "Chapter 5: Vision-Language-Action" which showed 404 errors instead of navigating to the chapter.

Command: /sp.git.commit_pr

## Response snapshot

Fixed the navigation bug in ChunkDisplay.tsx by adding a CHAPTER_PATH_MAP lookup table that maps chapter title variations to their correct Docusaurus doc paths. The slugify function was generating incorrect URLs (e.g., /docs/chapter-2-ros-2-fundamentals) that didn't match the actual folder structure (e.g., /docs/chapter-2-ros2).

Key changes:
- Added CHAPTER_PATH_MAP with mappings for all 6 chapters
- Added getChapterPath() function with exact and partial matching
- Updated handleViewInBook() to use the new path mapping

Committed and pushed to feat/book-rag-chatbot-system branch.

## Outcome

- ✅ Impact: Fixed 404 errors on View in book links, improving user navigation experience
- 🧪 Tests: Build passes successfully, manual navigation testing required
- 📁 Files: 12 files changed, 529 insertions, 59 deletions
- 🔁 Next prompts: Verify navigation works for all chapter links
- 🧠 Reflection: URL path mapping needed because RAG metadata uses human-readable chapter titles that don't match the folder structure

## Evaluation notes (flywheel)

- Failure modes observed: None - fix was straightforward path mapping
- Graders run and results (PASS/FAIL): Build PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider extracting path mapping to a shared config
