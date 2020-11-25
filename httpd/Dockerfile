FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    haproxy \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

COPY ./httpd app/
WORKDIR /app

CMD ["/usr/sbin/haproxy", "-f", "/app/haproxy.conf"]