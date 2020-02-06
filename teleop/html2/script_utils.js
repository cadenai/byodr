
function extend(proto, literal) {
    var result = Object.create(proto);
    Object.keys(literal).forEach(function(key) {
        result[key] = literal[key];
    });
    return result;
}

var socket_utils = {
    create_socket: function(path, binary=true) {
        ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
        web_socket = new WebSocket(ws_protocol + document.location.hostname + ":" + document.location.port + path);
        if (binary) {
            web_socket.binaryType = 'arraybuffer';
        }
        return web_socket;
    }
}