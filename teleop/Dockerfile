FROM byodr/bionic:1.0

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

EXPOSE 9100