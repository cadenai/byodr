"use strict";

const http               = require('http');
const express            = require('express');
const RemoteTCPFeedRelay = require('./remotetcpfeed');
const app                = express();

const IMG_WIDTH = process.env.IMG_WIDTH || 1280;
const IMG_HEIGHT = process.env.IMG_HEIGHT || 720;
const FEED_IP = process.env.FEED_IP || 'localhost';
const FEED_PORT = process.env.FEED_PORT || 5101;
const SERVER_PORT = process.env.SERVER_PORT || 9101;

const server  = http.createServer(app);

const feed    = new RemoteTCPFeedRelay(server, {
  width : IMG_WIDTH,
  height: IMG_HEIGHT,
  feed_ip   : FEED_IP,
  feed_port : FEED_PORT,
});

server.listen(SERVER_PORT);


