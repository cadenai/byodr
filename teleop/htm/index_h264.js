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

                $("canvas#h264_front_camera_main_image").click(function() {
                    if (teleop_screen.is_camera_selected(0)) {
                        teleop_screen.select_camera(-1);
                    } else {
                        teleop_screen.select_camera(0);
                    }
                });
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
                this.socket.attempt_reconnect = false;
                this.wsavc.disconnect();
                this.socket = null;
            }
        }
    }

    teleop_screen.add_camera_selection_listener(function(previous_id, current_id) {
        $("canvas#h264_front_camera_main_image").removeClass('selected');
        if (current_id == 0) {
            $("canvas#h264_front_camera_main_image").addClass('selected');
        }
    });
}

function h264_start_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.init(document.getElementById('viewport_container'));
        camera_controller.start(9101);
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
