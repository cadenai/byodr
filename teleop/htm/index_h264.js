if (page_utils.get_stream_type() == 'h264') {
    class CameraSocketResumer {
        constructor(uri, reconnect_ms) {
            this.uri = uri;
            this.reconnect_ms = reconnect_ms;
            this.attemt_reconnect = true;
        }

        onopen(player) {
            player.playStream();
        }

        onclose(player) {
            if (this.attemt_reconnect) {
                setTimeout(function() {player.connect(this.uri);}, this.reconnect_ms);
            }
        }
    }

    var camera_player = {
        el_canvas: null,
        wsavc: null,
        socket: null,

        init: function(el_parent) {
            if (this.el_canvas == undefined) {
                this.el_canvas = document.createElement("canvas");
                el_parent.appendChild(this.el_canvas)
            }
        },
        start: function(port) {
            uri = "ws://" + document.location.hostname + ':' + port;
            this.socket = new CameraSocketResumer(uri, 100);
            this.wsavc = new WSAvcPlayer(this.el_canvas, "webgl", this.socket);
            this.wsavc.connect(uri);
        },
        stop: function() {
            if (this.socket != undefined) {
                this.socket.attemt_reconnect = false;
                this.wsavc.disconnect();
                this.socket = null;
            }
        }
    }
}

function h264_start_all() {
    if (camera_player != undefined && camera_player.socket == undefined) {
        camera_player.init(document.getElementById('camera1'));
        camera_player.start(9101);
    }
}

function h264_stop_all() {
    if (camera_player != undefined) {
        camera_player.stop();
    }
}
