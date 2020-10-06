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
        el_image_preview: null,
        preview_timer: null,

        init: function(el_parent, el_image_preview) {
            if (this.el_canvas == undefined) {
                this.el_canvas = document.createElement("canvas");
                el_parent.appendChild(this.el_canvas)
            }
            this.el_image_preview = el_image_preview;
        },
        render_preview: function() {
            if (this.el_canvas != undefined) {
                this.el_image_preview.src = this.el_canvas.toDataURL("image/jpeg", 0.95);
            }
            this.preview_timer = setTimeout(function() {camera_controller.render_preview();}, 500);
        },
        start: function(port) {
            uri = "ws://" + document.location.hostname + ':' + port;
            this.socket = new CameraSocketResumer(uri, 100);
            this.wsavc = new WSAvcPlayer(this.el_canvas, "yuv", this.socket);
            this.wsavc.connect(uri);
        },
        stop: function() {
            if (this.socket != undefined) {
                this.socket.attempt_reconnect = false;
                this.wsavc.disconnect();
                this.socket = null;
            }
            if (this.preview_timer != undefined) {
                clearTimeout(this.preview_timer);
            }
        }
    }
}

function h264_start_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.init(document.getElementById('cameras_container'), document.getElementById('camera0_preview'));
        camera_controller.start(9101);
        camera_controller.render_preview();
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
