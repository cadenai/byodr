"use strict";

const http               = require('http');
const express            = require('express');
const RemoteTCPFeedRelay = require('./remotetcpfeed');
const app                = express();

var propertiesReader = require('properties-reader');
// When duplicate names are found in the properties, the first one read will be replaced with the later one.
var properties = propertiesReader('/app/config.ini');
properties.append('/config/config.ini')

const IMG_WIDTH = properties.get('camera.input.stream.image.width');
const IMG_HEIGHT = properties.get('camera.input.stream.image.height');
const FEED_IP = properties.get('camera.input.stream.feed.ip');
const FEED_PORT = properties.get('camera.input.stream.feed.port');
const SERVER_PORT = properties.get('camera.output.stream.server.port');

const server  = http.createServer(app);

const feed    = new RemoteTCPFeedRelay(server, {
  width : IMG_WIDTH,
  height: IMG_HEIGHT,
  feed_ip   : FEED_IP,
  feed_port : FEED_PORT,
});

server.listen(SERVER_PORT);


