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
                // The live-player must be told the video dimensions with the canvas width and height attributes.
                this.el_canvas = document.createElement("canvas");
                this.el_canvas.id = 'h264_camera_main_image';
                this.el_canvas.width = 640;
                this.el_canvas.height = 480;
                this.el_canvas.style.cssText = 'width: 100% !important; height: 100% !important;';
                el_parent.appendChild(this.el_canvas)
            }
        },
        start: function(camera_position) {
            const port = camera_position == 'front' ? 9001 : 9002;
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
                delete this.wsavc.ws;
                delete this.wsavc;
            }
        }
    }

    teleop_screen.add_camera_activation_listener(function(position) {
        camera_controller.stop();
        camera_controller.start(position);
    });
}

function h264_start_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.init(document.getElementById('viewport_container'));
        camera_controller.start(teleop_screen.active_camera);
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
