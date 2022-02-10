FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    haproxy \
    npm \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN npm install -g mapport

COPY ./httpd app/
WORKDIR /app
