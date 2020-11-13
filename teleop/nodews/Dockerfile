#FROM node
FROM centipede2donald/byodr-ce:node-h264ws

COPY ./teleop/nodews app/
WORKDIR /app

#RUN npm install

CMD ["node", "server-tcp.js"]