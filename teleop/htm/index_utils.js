
function extend(proto, literal) {
    var result = Object.create(proto);
    Object.keys(literal).forEach(function(key) {
        result[key] = literal[key];
    });
    return result;
}

var socket_utils = {
    create_socket: function(path, binary=true, reconnect=1000, assign=function(e) {}) {
        ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
        ws_url = ws_protocol + document.location.hostname + ":" + document.location.port + path;
        ws = new WebSocket(ws_url);
        if (binary) {
            ws.binaryType = 'arraybuffer';
        }
        assign(ws);
        var on_close = ws.onclose;
        ws.onclose = function() {
            setTimeout(function() {socket_utils.create_socket(path, binary, reconnect, assign);}, reconnect);
            if (typeof on_close === "function") {
                on_close();
            }
        };
    }
}

var page_utils = {
    get_stream_type: function() {
        var stream_type = window.localStorage.getItem('video.stream.type');
        if (stream_type == null) {
            return 'mjpeg';
        } else {
            return stream_type;
        }
    },

    set_stream_type: function(stream_type) {
        window.localStorage.setItem('video.stream.type', stream_type);
    }
}
