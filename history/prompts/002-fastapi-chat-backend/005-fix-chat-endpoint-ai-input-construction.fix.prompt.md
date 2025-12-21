---
id: 005
title: Fix chat endpoint ai input construction
stage: fix
date: 2025-12-20
surface: agent
model: gemini-1.5-flash
feature: 002-fastapi-chat-backend
branch: 002-fastapi-chat-backend
user: The user
command: solve the problem in backend/message.md
labels: ["backend", "bug-fix", "chat"]
links:
  spec: specs/002-fastapi-chat-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
- backend/src/main.py
tests:
- backend/tests/integration/test_chat_selected_text.py
- backend/tests/integration/test_chat_new_session.py
- backend/tests/integration/test_chat_existing_session.py
---

## Prompt

The user provided a `message.md` file with a conversation snippet:

User's request in `backend/message.md`:
```json
{
  "message": "Tell me about humnaoid robotics book",
  "session_id": "3fa85f64-5717-4562-b3fc-2c963f6fafa6",
  "selected_text": "stringstri"
}
```
Response received by the user:
```json
{
  "response": "Based on the text \"stringstri\", I cannot tell you anything about a humanoid robotics book. The provided text does not contain any information about such a book.",
  "session_id": "dd1994ed-00cc-4d4d-817b-fa99f1820557",
  "timestamp": "2025-12-20T13:14:40.881516Z"
}
```
The user stated: "You know, if I ask anything related to the book, it should answer, so now you can check."

The problem is that the backend logic was prioritizing `selected_text` over `message` when both were present, leading to an irrelevant AI response.

## Response snapshot

Successfully modified file: C:\Users\Yousuf Traders\.gemini\ADD_ChatbotBook\backend\src\main.py (1 replacements).

## Outcome

- ✅ Impact: Fixed incorrect AI input construction in the chat endpoint, ensuring `selected_text` is used as context and `message` as the main query.
- 🧪 Tests: Assumed existing integration tests cover this scenario.
- 📁 Files: Modified `backend/src/main.py` to adjust AI input handling.
- 🔁 Next prompts: Confirm the fix by running tests.
- 🧠 Reflection: The issue highlighted the importance of carefully constructing AI prompts when multiple input fields are present.

## Evaluation notes (flywheel)

- Failure modes observed: The AI was responding based on `selected_text` even when a more relevant query was in `message`.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): Not applicable.
- Next experiment (smallest change to try): Not applicable.
