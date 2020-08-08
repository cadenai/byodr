FROM node

COPY ./teleop/nodews app/
WORKDIR /app

RUN npm install

#RUN groupadd --gid 1990 --system byodr \
#  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

#USER byodr

CMD ["node", "server-tcp.js"]