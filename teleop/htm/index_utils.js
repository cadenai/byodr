jQuery.fn.visible = function() {
    return this.css('visibility', 'visible');
};

jQuery.fn.invisible = function() {
    return this.css('visibility', 'hidden');
};

jQuery.fn.is_visible = function() {
    return this.css('visibility') == 'visible';
};

function extend(proto, literal) {
    var result = Object.create(proto);
    Object.keys(literal).forEach(function(key) {
        result[key] = literal[key];
    });
    return result;
}

var socket_utils = {
    create_socket: function(path, binary=true, reconnect=100, assign=function(e) {}) {
        ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
        ws_url = ws_protocol + document.location.hostname + ":" + document.location.port + path;
        ws = new WebSocket(ws_url);
        if (binary) {
            ws.binaryType = 'arraybuffer';
        }
        assign(ws);
        ws.onclose = function() {
            if (typeof ws.is_reconnect == "function" && ws.is_reconnect()) {
                setTimeout(function() {socket_utils.create_socket(path, binary, reconnect, assign);}, reconnect);
            }
            if (typeof ws.on_close === "function") {
                ws.on_close();
            }
        };
    }
}

var page_utils = {
    system_capabilities: {},

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

jQuery.get("/api/system/capabilities", function(data) {
    page_utils.system_capabilities = data;
});