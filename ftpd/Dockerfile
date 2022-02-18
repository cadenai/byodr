# https://hub.docker.com/r/gists/pure-ftpd/dockerfile
FROM gists/pure-ftpd:1.0.49

ENV PYTHONUNBUFFERED=1
RUN apk add --update --no-cache python3 && ln -sf python3 /usr/bin/python

ENV PUBLIC_HOST=localhost \
    MIN_PASV_PORT=30000 \
    MAX_PASV_PORT=30009 \
    UID=1000 \
    GID=1000

VOLUME /home/ftpuser /etc/pureftpd

EXPOSE 21 $MIN_PASV_PORT-$MAX_PASV_PORT

COPY ./ftpd app/
WORKDIR /app

ENTRYPOINT ["/usr/bin/entrypoint.sh"]

CMD /usr/sbin/pure-ftpd \
        -P $PUBLIC_HOST \
        -p $MIN_PASV_PORT:$MAX_PASV_PORT \
        -l puredb:/etc/pureftpd/pureftpd.pdb \
        -E \
        -j \
        -R