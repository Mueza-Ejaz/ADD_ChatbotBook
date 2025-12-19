$env:Path += ";C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts"
& "C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts\poetry" init --name fastapi-chat-backend --python "3.12" --no-interaction
& "C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts\poetry" env use python
& "C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts\poetry" add fastapi uvicorn google-generativeai sqlalchemy alembic pydantic python-dotenv asyncpg