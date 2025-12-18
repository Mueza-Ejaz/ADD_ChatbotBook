# Use a Python 3.12 slim image as the base
FROM python:3.12-slim-bookworm

# Set the working directory in the container
WORKDIR /app/backend

# Install Poetry
RUN pip install poetry

# Copy pyproject.toml and poetry.lock to the working directory
COPY pyproject.toml poetry.lock ./

# Install project dependencies
RUN poetry install --no-root --no-interaction --no-ansi

# Copy the rest of the application code
COPY src ./src
COPY alembic ./alembic

# Expose the port FastAPI runs on
EXPOSE 8000

# Command to run the application using uvicorn
CMD ["poetry", "run", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]