import os
import sys

def validate_env():
    """
    Validates the presence of required environment variables.
    Exits with an error if any essential variable is missing.
    """
    required_vars = ["DATABASE_URL", "GEMINI_API_KEY"]
    missing_vars = []

    print("Validating environment variables...")

    for var in required_vars:
        if os.getenv(var) is None:
            missing_vars.append(var)

    if missing_vars:
        print("\nError: The following required environment variables are not set:")
        for var in missing_vars:
            print(f"- {var}")
        print("\nPlease set these variables in your .env file or environment.")
        sys.exit(1)
    else:
        print("All required environment variables are set.")
        sys.exit(0)

if __name__ == "__main__":
    from dotenv import load_dotenv
    # Load .env file if it exists
    dotenv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '.env')
    if os.path.exists(dotenv_path):
        load_dotenv(dotenv_path)
    
    validate_env()
