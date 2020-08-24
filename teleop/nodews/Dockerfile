FROM node

COPY ./teleop/nodews app/
WORKDIR /app

RUN npm install

CMD ["node", "server-tcp.js"]