---
id: 003
title: Fix CORS error for localhost:3001 frontend
stage: general
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: feat/book-rag-chatbot-system
user: Kashaf Aman
command: sp.analyze
labels: ["cors", "debugging", "api", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - Backend/api.py
tests:
  - none
---

## Prompt

Request URL http://localhost:8000/ask Referrer Policy strict-origin-when-cross-origin accept */* accept-encoding gzip, deflate, br, zstd accept-language en-GB,en-US;q=0.9,en;q=0.8 connection keep-alive content-length 17 content-type application/json host localhost:8000 origin http://localhost:3001 referer http://localhost:3001/ sec-fetch-dest empty sec-fetch-mode cors sec-fetch-site same-site user-agent Mozilla/5.0 (iPhone; CPU iPhone OS 18_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/18.5 Mobile/15E148 Safari/604.1 solve this error

## Response snapshot

Diagnosed CORS error: Frontend at localhost:3001 was blocked from accessing backend at localhost:8000 because the ALLOWED_ORIGINS in api.py only included localhost:3000 by default.

Fixed by updating Backend/api.py:98 to include localhost:3001 and 127.0.0.1:3001 in the CORS allowed origins:

```python
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000,http://localhost:3001,http://127.0.0.1:3001,http://localhost:8000,http://127.0.0.1:8000").split(",")
```

## Outcome

- Impact: Fixed CORS blocking issue allowing frontend-backend communication
- Tests: Manual verification required - restart backend and retry request
- Files: Backend/api.py (line 98 modified)
- Next prompts: Verify fix works after server restart
- Reflection: CORS origin mismatch is a common issue when frontend runs on a different port than expected

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward config fix
- Graders run and results (PASS/FAIL): N/A - manual verification needed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
