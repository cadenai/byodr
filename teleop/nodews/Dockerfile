FROM node:12.20.0-alpine3.11

COPY ./teleop/nodews app/
WORKDIR /app

RUN npm install

CMD ["node", "server-tcp.js"]