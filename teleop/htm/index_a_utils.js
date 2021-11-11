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
        var _assigned_on_close = ws.onclose;
        ws.onclose = function() {
            if (typeof ws.is_reconnect == "function" && ws.is_reconnect()) {
                setTimeout(function() {socket_utils.create_socket(path, binary, reconnect, assign);}, reconnect);
            }
            if (typeof _assigned_on_close === "function") {
                _assigned_on_close();
            }
        };
    }
}

var dev_tools = {
    _develop: null,

    _parse_develop: function() {
        const params = new URLSearchParams(window.location.search);
        var _develop = params.get('develop');
        if (_develop != undefined) {
            _develop = ['xga', 'svga', 'vga'].includes(_develop)? _develop: 'xga';
        }
        return _develop;
    },

    _init: function() {
        this._develop = this._parse_develop();
    },

    is_develop: function() {
        return this._develop == undefined? this._parse_develop(): this._develop;
    },

    get_img_dimensions: function() {
        const _key = this.is_develop();
        switch(_key) {
            case "xga":
                return [1024, 768];
            case "svga":
                return [800, 600];
            case "vga":
                return [640, 480];
            default:
                return [320, 240];
        }
    },

    get_img_url: function(camera_position) {
        const _key = this.is_develop();
        return '/develop/img_' + camera_position + '_' + _key + '.jpg';
    },

    set_next_resolution: function() {
        var _next = null;
        const _key = this.is_develop();
        switch(_key) {
            case "xga":
                _next = 'svga';
                break;
            case "svga":
                _next = 'vga';
                break;
            case "vga":
                _next = 'qvga';
                break;
            default:
                _next = 'xga';
        }
        this._develop = _next;
    }
}

var page_utils = {
    system_capabilities: {},

    _init: function() {
        if (dev_tools.is_develop()) {
            console.log("Development mode is active.")
            page_utils.system_capabilities = {
                'platform': {'video': {'front': {'ptz': 0}, 'rear': {'ptz': 0}}}
            };
        } else {
            jQuery.get("/api/system/capabilities", function(data) {
                page_utils.system_capabilities = data;
            });
        }
    },

    get_stream_type: function() {
        var stream_type = window.localStorage.getItem('video.stream.type');
        if (stream_type == null) {
            return 'h264';
        } else {
            return stream_type;
        }
    },

    set_stream_type: function(stream_type) {
        window.localStorage.setItem('video.stream.type', stream_type);
    }
}


// --------------------------------------------------- Initialisations follow --------------------------------------------------------- //
dev_tools._init();
page_utils._init();