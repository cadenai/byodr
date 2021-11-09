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

    var canvas_controller = {
        el_parent: null,
        el_canvas_id: 'viewport_canvas',
        el_canvas: null,
        context_2d: null,

        init: function(el_parent) {
            this.el_parent = el_parent;
        },
        replace: function(canvas) {
            if (this.el_canvas != undefined) {
                this.el_canvas.remove();
            }
            this.context_2d = canvas.getContext('2d');
            this.el_canvas = canvas;
            this.el_parent.appendChild(this.el_canvas);
        },
        create: function() {
            canvas = document.getElementById(this.el_canvas_id);
            // The canvas dimensions are set by the player.
            // canvas = document.createElement("canvas");
            // canvas.id = this.el_canvas_id;
            // canvas.style.cssText = 'width: 100% !important; height: 100% !important;';
            return canvas;
        }
    }

    var camera_controller = {
        wsavc: null,
        socket: null,

        start: function(camera_position) {
            const port = camera_position == 'front' ? 9001 : 9002;
            const ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
            const uri = ws_protocol + document.location.hostname + ':' + port;
            this.socket = new CameraSocketResumer(uri, 100);
            // The webgl context does not have a 2d rendering context.
            this.wsavc = new WSAvcPlayer(canvas_controller.create(), "yuv", this.socket);
            this.wsavc.on('canvasReady', function(width, height) {
                // console.log("Canvas Ready.");
                canvas_controller.replace(camera_controller.wsavc.canvas);
            });
            this.wsavc.on('canvasRendered', function() {
                // Do not run the canvas draws in parallel.
                if (canvas_controller.context_2d != undefined) {
                    teleop_screen.canvas_update(canvas_controller.context_2d);
                }
            });
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
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && canvas_controller != undefined
        && camera_controller.socket == undefined) {
            canvas_controller.init(document.getElementById('viewport_container'));
            camera_controller.start(teleop_screen.active_camera);
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
