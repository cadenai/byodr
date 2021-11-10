if (page_utils.get_stream_type() == 'h264') {
    var camera_controller = {
        el_video: null,
        jmuxer: null,
        socket: null,

        init: function(el_parent) {
            if (this.el_video == undefined) {
                this.el_video = document.createElement("video");
                this.el_video.id = 'h264_camera_main_image';
                this.el_video.autoplay = true;
                this.el_video.muted = true; // Required for auto-play to start playing immediately in the browser.
                this.el_video.style.cssText = 'width: 100% !important; height: 100% !important;';
                el_parent.appendChild(this.el_video)
                this.jmuxer = new JMuxer({
                    node: 'h264_camera_main_image',
                    mode: 'video',
                    flushingTime: 0,
                    fps: 25,
                    debug: false,
                    onError: function(data) {
                        if (/Safari/.test(navigator.userAgent) && /Apple Computer/.test(navigator.vendor)) {
                            jmuxer.reset();
                        }
                    }
                 });
            }
        },
        connect: function() {
            const position = teleop_screen.active_camera;
            const port = position == 'front' ? 9001 : 9002;
            const ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
            const uri = ws_protocol + document.location.hostname + ':' + port;
            this.socket = new WebSocket(uri);
            this.socket.binaryType = 'arraybuffer';
            this.socket.attempt_reconnect = true;
            this.socket.addEventListener('message', function(event) {
                camera_controller.jmuxer.feed({video: new Uint8Array(event.data)});
            });
            this.socket.onclose = function() {
                if (this.attempt_reconnect) {
                    setTimeout(function() {camera_controller.connect();}, 100);
                }
            };
            //console.log("Jmuxer " + position + " camera connection established.");
        },
        start: function() {
            this.connect();
        },
        stop: function() {
            if (this.socket != undefined) {
                this.socket.attempt_reconnect = false;
                this.socket.close();
                this.jmuxer.reset();
                this.socket = null;
                delete this.socket;
                //console.log("Jmuxer camera connection closed.");
            }
        }
    }

    teleop_screen.add_camera_activation_listener(function(position) {
        camera_controller.stop();
        camera_controller.start();
    });
}

function h264_start_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.init(document.getElementById('viewport_container'));
        camera_controller.start();
    }
}

function h264_stop_all() {
    if (page_utils.get_stream_type() == 'h264' && camera_controller != undefined) {
        camera_controller.stop();
    }
}
