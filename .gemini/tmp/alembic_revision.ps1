$env:Path += ";C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts"
$env:PYTHONPATH = "C:\Users\Yousuf Traders\.gemini\ADD_ChatbotBook\backend"
$env:DATABASE_URL = "postgresql+asyncpg://user:password@localhost/dbname" # Dummy URL for generation
& "C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts\poetry" run alembic revision -m "Create chat_sessions and messages tables"