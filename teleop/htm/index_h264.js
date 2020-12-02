if (page_utils.get_stream_type() == 'h264') {
    class CameraSocketResumer {
        constructor(uri, reconnect_ms) {
            this.uri = uri;
            this.reconnect_ms = reconnect_ms;
            this.attempt_reconnect = true;
        }
        onopen(player) {
            player.playStream();
        }
        onclose(player) {
            if (this.attempt_reconnect) {
                setTimeout(function() {player.connect(this.uri);}, this.reconnect_ms);
            }
        }
    }

    var camera_controller = {
        el_canvas: null,
        wsavc: null,
        socket: null,

        init: function(el_parent) {
            if (this.el_canvas == undefined) {
                this.el_canvas = document.createElement("canvas");
                this.el_canvas.id = 'h264_front_camera_main_image';
                el_parent.appendChild(this.el_canvas)
            }
        },
        start: function(port) {
            const ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
            const uri = ws_protocol + document.location.hostname + ':' + port;
            this.socket = new CameraSocketResumer(uri, 100);
            this.wsavc = new WSAvcPlayer(this.el_canvas, "webgl", this.socket);
            this.wsavc.connect(uri);
        },
        stop: function() {
            if (this.socket != undefined && this.wsavc != undefined) {
                this.socket.attempt_reconnect = false;
                this.wsavc.disconnect();
                this.socket = null;
            }
        }
    }

    teleop_screen.add_camera_selection_listener(function(current) {
        $("canvas#h264_front_camera_main_image").removeClass('selected');
        if (current == 'front') {
            $("canvas#h264_front_camera_main_image").addClass('selected');
        }
    });
}

function h264_start_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.init(document.getElementById('viewport_container'));
        camera_controller.start(9001);
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
