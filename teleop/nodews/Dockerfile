# docker build -t centipede2donald/byodr-ce:nodews -f teleop/nodews/Dockerfile .
FROM node

COPY ./teleop/nodews app/
WORKDIR /app

RUN npm install

CMD ["node", "server-tcp.js"]