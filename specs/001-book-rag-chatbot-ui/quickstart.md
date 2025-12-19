# Quickstart Guide: Book RAG Chatbot UI

**Feature**: 001-book-rag-chatbot-ui
**Date**: 2025-12-18
**Status**: Complete

## Purpose

This guide provides step-by-step instructions for setting up the development environment, running the chatbot UI locally, and testing the core functionality.

## Prerequisites

Before starting, ensure you have:

- **Node.js**: v20.0 or higher (required by Docusaurus)
- **Python**: 3.11 or higher (for FastAPI backend)
- **uv** (Python package manager): Installed via `pip install uv`
- **Git**: For cloning and version control
- **Code Editor**: VS Code recommended (with TypeScript and React extensions)

**Verify installations**:
```bash
node --version   # Should be v20.0+
python --version # Should be 3.11+
uv --version     # Should show uv version
```

## Project Structure Overview

```
book-speckitplus/
├── Backend/              # FastAPI backend (EXISTING - no changes)
│   ├── api.py           # /ask endpoint
│   ├── agent_rag.py     # RAG orchestrator
│   └── models/
│       └── agent_models.py
├── my-book/             # Docusaurus frontend (NEW chatbot UI here)
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatbotUI/  # NEW components
│   │   ├── hooks/          # NEW hooks
│   │   ├── services/       # NEW API client
│   │   ├── types/          # NEW TypeScript types
│   │   └── theme/
│   │       └── Root.tsx    # NEW global wrapper
│   └── package.json
└── specs/
    └── 001-book-rag-chatbot-ui/  # This planning documentation
```

## Setup Instructions

### Step 1: Install Backend Dependencies

The backend is already set up, but verify it's working:

```bash
# Navigate to backend directory
cd Backend

# Activate virtual environment (if using uv)
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
uv pip install -r requirements.txt  # Or use existing uv.lock

# Verify installation
python -c "import fastapi; print('FastAPI OK')"
python -c "import qdrant_client; print('Qdrant OK')"
python -c "import cohere; print('Cohere OK')"
```

**Expected output**: "FastAPI OK", "Qdrant OK", "Cohere OK"

### Step 2: Configure Backend Environment

Ensure `.env` file exists in `Backend/` with required API keys:

```bash
# Backend/.env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key  # For agent
```

**Verify configuration**:
```bash
cat .env  # Check keys are present (don't share these!)
```

### Step 3: Start Backend Server

```bash
# From Backend/ directory
python start_server.py

# Expected output:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Application startup complete.
```

**Test backend**:
```bash
# In a new terminal
curl http://localhost:8000/health

# Expected response:
# {"status":"healthy","message":"Book RAG Chatbot API is running"}
```

**Important**: Leave this terminal running. Backend must be active for UI testing.

### Step 4: Install Frontend Dependencies

```bash
# Navigate to frontend directory
cd my-book

# Install dependencies
npm install

# Verify installation
npm list react react-dom @docusaurus/core

# Expected: All packages listed with versions
```

**Note**: This installs existing Docusaurus dependencies. New chatbot components will be added during implementation.

### Step 5: Start Frontend Development Server

```bash
# From my-book/ directory
npm start

# Expected output:
# [SUCCESS] Docusaurus website is running at http://localhost:3000/
```

**Access the book**: Open http://localhost:3000 in your browser.

**Note**: In the current state (before implementation), the chatbot UI will not be visible. This step verifies the base Docusaurus setup works.

---

## Development Workflow (After Implementation)

Once the chatbot UI is implemented (via `/sp.tasks`), follow this workflow:

### Starting the Application

**Terminal 1 - Backend**:
```bash
cd Backend
source .venv/bin/activate  # Activate Python env
python start_server.py     # Start FastAPI server
```

**Terminal 2 - Frontend**:
```bash
cd my-book
npm start  # Start Docusaurus dev server
```

**Access**: http://localhost:3000

### Testing the Chatbot UI

#### Test 1: Basic Question and Answer

1. **Open book** at http://localhost:3000
2. **Look for floating icon** (💬) in bottom-right corner
3. **Click icon** to open chatbot panel
4. **Enter question**: "What are the key features of ROS 2?"
5. **Click Send** (or press Enter)
6. **Verify**:
   - Loading indicator appears within 100ms
   - Answer displayed within 5 seconds
   - Answer includes source references (chapter/section links)
   - Matched chunks shown below answer
   - Source links are clickable

**Expected behavior**: Panel opens, question sent, answer displayed with sources and chunks.

#### Test 2: Selected Text Query

1. **Select text** on book page (highlight a paragraph)
2. **Open chatbot** (💬 icon)
3. **Verify**: Selected text badge shows (e.g., "Selected text: Real-time systems require...")
4. **Enter question**: "Explain this passage"
5. **Click Send**
6. **Verify**:
   - Loading indicator appears
   - Answer based on selected text only
   - Sources list shows "Selected Text"
   - Matched chunk displays the selection

**Expected behavior**: Answer uses only the selected text as context.

#### Test 3: Empty Query Validation

1. **Open chatbot**
2. **Leave input empty** (or type only whitespace)
3. **Click Send**
4. **Verify**: Send button disabled OR validation message shown

**Expected behavior**: Cannot submit empty query.

#### Test 4: No Results (Refusal)

1. **Open chatbot**
2. **Ask irrelevant question**: "What is quantum physics?" (assuming book is not about physics)
3. **Click Send**
4. **Verify**:
   - Answer displayed: "This information is not available in the book."
   - No sources or chunks shown
   - Helpful message suggests reformulating query

**Expected behavior**: Clear refusal message without error.

#### Test 5: Error Handling (Backend Offline)

1. **Stop backend server** (Ctrl+C in Terminal 1)
2. **Open chatbot**
3. **Ask question**: "What is ROS 2?"
4. **Click Send**
5. **Verify**:
   - Error message displayed: "Unable to connect. Check your internet connection."
   - No technical error details or stack traces

**Expected behavior**: User-friendly error message.

#### Test 6: Source Navigation

1. **Ask question** that returns answer with sources
2. **Click a source link** (e.g., "Chapter 3: ROS 2 Overview")
3. **Verify**:
   - Browser navigates to book section
   - Chatbot panel remains open (or can be reopened)
   - Navigation smooth, no page reload

**Expected behavior**: Clicking source navigates to chapter/section.

#### Test 7: Panel Minimize/Maximize

1. **Open chatbot panel**
2. **Click X or minimize button**
3. **Verify**: Panel closes, icon remains
4. **Click icon again**
5. **Verify**: Panel reopens, conversation history preserved

**Expected behavior**: Panel can be closed and reopened without losing history.

#### Test 8: Navigation Persistence

1. **Open chatbot, ask question**
2. **Navigate to different book page** (click chapter link in sidebar)
3. **Reopen chatbot**
4. **Verify**: Previous conversation visible

**Expected behavior**: Conversation history persists across page navigation.

---

## Manual Testing Checklist

Use this checklist before marking implementation complete:

### Functional Requirements (FR-001 to FR-013)

- [ ] FR-001: Input interface visible and accessible on all book pages
- [ ] FR-002: Query sent to backend `/ask`, answer displayed
- [ ] FR-003: Answer, sources, and matched chunks all displayed
- [ ] FR-004: Selected text automatically detected and captured
- [ ] FR-005: Selected text sent as `selected_text` parameter to backend
- [ ] FR-006: Loading indicator shown during processing
- [ ] FR-007: Empty results handled gracefully with helpful message
- [ ] FR-008: Network/backend errors shown with user-friendly messages
- [ ] FR-009: Empty query submission prevented (disabled button or validation)
- [ ] FR-010: Source references clickable, navigate to book sections
- [ ] FR-011: Chatbot UI integrates without breaking book layout
- [ ] FR-012: Communication via REST API POST to `/ask` endpoint
- [ ] FR-013: Matched chunks displayed with text and source location

### Success Criteria (SC-001 to SC-010)

- [ ] SC-001: Query response in <5 seconds (95% of queries)
- [ ] SC-002: Loading indicator appears within 100ms
- [ ] SC-003: Empty result message displayed within 3 seconds
- [ ] SC-004: 100% of answers include sources and chunks
- [ ] SC-005: 100% of source clicks navigate correctly
- [ ] SC-006: Chatbot accessible across all book pages without reload
- [ ] SC-007: Selected text works for up to 2000 characters
- [ ] SC-008: 3+ concurrent queries handled without UI blocking
- [ ] SC-009: Error messages within 5s, no stack traces
- [ ] SC-010: UI doesn't cause scroll or overlap critical content

---

## Common Issues & Troubleshooting

### Issue: Backend not starting

**Symptom**: `python start_server.py` fails

**Solutions**:
1. Check `.env` file exists with valid API keys
2. Verify Python version: `python --version` (should be 3.11+)
3. Reinstall dependencies: `uv pip install -r requirements.txt`
4. Check port 8000 not in use: `lsof -i :8000` (kill conflicting process)

### Issue: Frontend not starting

**Symptom**: `npm start` fails

**Solutions**:
1. Delete `node_modules` and `package-lock.json`, run `npm install` again
2. Clear Docusaurus cache: `npm run clear`
3. Verify Node version: `node --version` (should be v20.0+)
4. Check port 3000 not in use

### Issue: Chatbot icon not visible

**Symptom**: No 💬 icon on book pages

**Solutions**:
1. Verify `src/theme/Root.tsx` exists and exports ChatbotProvider
2. Check browser console for React errors
3. Ensure `ChatbotToggle` component is rendered in Root.tsx
4. Hard refresh browser: Ctrl+F5 (Windows) or Cmd+Shift+R (Mac)

### Issue: "Unable to connect" error

**Symptom**: Every query shows connection error

**Solutions**:
1. Verify backend running: `curl http://localhost:8000/health`
2. Check CORS settings in `api.py` (should allow `http://localhost:3000`)
3. Check browser DevTools Network tab for request details
4. Verify API URL in `chatbotApi.ts` is `http://localhost:8000/ask`

### Issue: Selected text not captured

**Symptom**: Selected text not showing in chatbot

**Solutions**:
1. Check `useTextSelection.ts` hook is active
2. Verify `mouseup` event listener registered
3. Test with plain text (not code blocks or images)
4. Check browser console for JavaScript errors

### Issue: Source links not working

**Symptom**: Clicking source doesn't navigate

**Solutions**:
1. Verify `useHistory` from `@docusaurus/router` is imported
2. Check backend returns valid `chapter` and `section` in `ChunkReference`
3. Verify Docusaurus heading IDs match chapter structure
4. Test navigation manually: type `/docs/chapter-3#section` in browser

---

## Development Tools

### Recommended VS Code Extensions

- **ESLint**: JavaScript/TypeScript linting
- **Prettier**: Code formatting
- **TypeScript**: Enhanced TypeScript support
- **React Developer Tools** (browser extension): Debug React components

### Useful Commands

```bash
# Frontend (my-book/)
npm start          # Start dev server
npm run build      # Build for production
npm run serve      # Serve production build
npm run clear      # Clear Docusaurus cache
npm run typecheck  # Run TypeScript type checks

# Backend (Backend/)
python start_server.py     # Start FastAPI server
pytest tests/              # Run backend tests (after implementation)
python -m uvicorn api:app --reload  # Alternative server start with auto-reload
```

### Browser DevTools

- **Console**: Check for JavaScript errors
- **Network**: Inspect `/ask` API requests/responses
- **React DevTools**: Inspect component state and props
- **Elements**: Debug CSS and layout issues

---

## Next Steps

1. **Review this quickstart**: Ensure all prerequisites are met
2. **Run `/sp.tasks`**: Generate implementation tasks
3. **Implement P1 stories first**: Basic Q&A and selected text queries
4. **Test incrementally**: Use the testing checklist after each component
5. **Iterate on UX**: Adjust styling, positioning, loading indicators based on manual testing

---

## Additional Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **React 19 Docs**: https://react.dev
- **TypeScript Handbook**: https://www.typescriptlang.org/docs/
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Feature Spec**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contract**: [contracts/chatbot-ui-api.yaml](./contracts/chatbot-ui-api.yaml)
