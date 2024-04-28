# Use the official Python 3.8 image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
# Set the environment variable
ENV FLASK_ENV=development


# Add app files
COPY . .

# Expose port 5000
EXPOSE 5001

# Start the app
CMD ["python", "main.py"]
