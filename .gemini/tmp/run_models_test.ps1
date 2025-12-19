$env:Path += ";C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts"
$env:PYTHONPATH = "C:\Users\Yousuf Traders\.gemini\ADD_ChatbotBook\backend"
& "C:\Users\Yousuf Traders\AppData\Roaming\Python\Scripts\poetry" run pytest tests/unit/test_models.py