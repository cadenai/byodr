if (page_utils.get_stream_type() == 'mjpeg') {
    var mjpeg_controller = {
        actual_fps: 0,
        target_fps: 16,
        display_resolution: 'default',
        jpeg_quality: 20,
        min_jpeg_quality: 25,
        max_jpeg_quality: 50,

        init: function() {
            var _fps = window.localStorage.getItem('mjpeg.target.fps');
            if (_fps != null) {
                this.target_fps = JSON.parse(_fps);
            }
            var _quality_max = window.localStorage.getItem('mjpeg.quality.max');
            if (_quality_max != null) {
                this.set_max_quality(JSON.parse(_quality_max));
            }
        },

        set_target_fps: function(val) {
            if (val > 0) {
                this.target_fps = val;
                window.localStorage.setItem('mjpeg.target.fps', JSON.stringify(val));
            }
        },

        set_max_quality: function(val) {
            if (val > 0 && val <= 100) {
                this.max_jpeg_quality = val;
                window.localStorage.setItem('mjpeg.quality.max', JSON.stringify(val));
                // Modify the minimum quality in lockstep with the maximum.
                _min = val / 2.0;
                if (_min < 5) {
                    _min = 5;
                }
                this.min_jpeg_quality = _min;
            }
        },

        update_quality: function() {
            var q_step = Math.min(1, Math.max(-1, this.actual_fps - this.target_fps));
            this.jpeg_quality = Math.min(this.max_jpeg_quality, Math.max(this.min_jpeg_quality, this.jpeg_quality + q_step));
        }
    }
    mjpeg_controller.init();

    var camera_controller = {
        request_start: performance.now(),
        request_time: 0,
        request_timeout: 0,
        // larger = more smoothing
        request_time_smoothing: 0.01,
        request_target_timeout: 100,
        el_image: null,
        socket_close_timer_id: null,

        init: function(el_parent) {
            el_image = document.createElement("img");
            el_image.style.width = '1280px';
            el_image.style.height = '720px';
            el_image.style.border = '1px solid white';
            el_image.style.borderRadius = '5px';
            el_parent.appendChild(el_image);
            this.el_image = el_image
            this.request_target_timeout = 1000. / mjpeg_controller.target_fps
            this.request_start = performance.now();
        },

        clear_socket_timeout: function () {
            if (this.socket_close_timer_id != undefined) {
                clearTimeout(this.socket_close_timer_id);
            }
        },

        update_framerate: function() {
            var end_time = performance.now();
            var duration = end_time - this.request_start;
            this.request_start = end_time;
            // smooth with moving average
            this.request_time = (this.request_time * this.request_time_smoothing) + (duration * (1.0 - this.request_time_smoothing));
            this.request_timeout = Math.max(0, this.request_target_timeout - this.request_time);
            mjpeg_controller.actual_fps = Math.round(1000 / this.request_time);
        }
    }

    camera_controller.socket_close = function(socket) {
        socket.close(4001, "Done waiting for the server to respond.");
        camera_controller.update_framerate();
    }
    camera_controller.capture = function(socket) {
        camera_controller.clear_socket_timeout();
        camera_controller.socket_close_timer_id = setTimeout(function() {camera_controller.socket_close(socket);}, 1000);
        // E.g. '{"quality": 50, "display": "vga"}'
        socket.send(JSON.stringify({
            quality: mjpeg_controller.jpeg_quality,
            display: mjpeg_controller.display_resolution
        }));
    };
    camera_controller.start_socket = function() {
        socket_utils.create_socket("/ws/cam", true, 100, function(ws) {
            camera_controller.socket = ws;
            ws.attempt_reconnect = true;
            ws.is_reconnect = function() {
                return ws.attempt_reconnect;
            }
            ws.onopen = function() {
                console.log("MJPEG socket connection established.");
                camera_controller.capture(ws);
            };
            ws.onclose = function() {
                console.log("MJPEG socket connection closed.");
            };
            ws.onmessage = function(evt) {
                camera_controller.clear_socket_timeout();
                camera_controller.update_framerate();
                mjpeg_controller.update_quality();
                camera_controller.el_image.src = window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"}));
                $('span#frame_fps').text(mjpeg_controller.actual_fps);
                $('span#frame_quality').text(mjpeg_controller.jpeg_quality);
                setTimeout(function() {camera_controller.capture(ws);}, camera_controller.request_timeout);
            };
        });
    };
    camera_controller.stop_socket = function() {
        camera_controller.socket.attempt_reconnect = false;
        if (camera_controller.socket.readyState < 2) {
            camera_controller.socket.close();
        }
        camera_controller.socket = null;
    };

    document.addEventListener("DOMContentLoaded", function() {
        camera_controller.init(document.getElementById('camera1'));
    });
}

function mjpeg_start_all() {
    if (camera_controller != undefined && camera_controller.socket == undefined) {
        camera_controller.start_socket();
    }
}

function mjpeg_stop_all() {
    if (camera_controller != undefined && camera_controller.socket != undefined) {
        camera_controller.stop_socket();
    }
}
