# Use the official Python 3.8 image
FROM python:3.11-slim

WORKDIR /app

EXPOSE 5001

ENV FLASK_APP=app.py

COPY . /app
RUN pip install -r requirements.txt

ENTRYPOINT [ "flask"]
CMD [ "run", "--host", "0.0.0.0" ]