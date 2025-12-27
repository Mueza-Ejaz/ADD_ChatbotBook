---
id: 1703816600
title: Clarify selected text functionality
stage: general
date: 2025-12-27
surface: agent
model: gemini-1.5-pro
feature: 005-rag-qdrant
branch: 005-rag-qdrant
user: 
command: to ab user koi bhi text ko book ka select kary ga to jawaab aiga ?
labels: ["clarification", "rag", "frontend-backend-interaction"]
links: 
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - N/A
tests_yaml: |
  - N/A
prompt_text: |
  to ab user koi bhi text ko book ka select kary ga to jawaab aiga ?
response_text: |
  Yes, that's correct. If a user selects text from the book, that selected text will be passed to the RAG system as a 'hint'.

  The system will use this hint to prioritize and retrieve more relevant sections from the book's content stored in Qdrant. The retrieved information, along with the user's question, will then be used to generate a more accurate and context-specific answer from the AI model.
outcome: |
  Clarified to the user that selected text will act as a hint for the RAG system, improving context retrieval and AI response accuracy.
evaluation: |
  Successfully clarified the functionality to the user.
---
