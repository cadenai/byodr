FROM mongo:4.4.12

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -f -y \
    python3 \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

COPY ./mongodb app/
WORKDIR /app
