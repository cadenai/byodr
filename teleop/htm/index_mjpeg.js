class MJPEGControlLocalStorage {
    constructor() {
        this.min_jpeg_quality = 25;
        this.max_jpeg_quality = 50;
        this.load();
    }
    set_max_quality(val) {
        if (val > 0 && val <= 100) {
            this.max_jpeg_quality = val;
            // Modify the minimum quality in lockstep with the maximum.
            var _min = val / 2.0;
            if (_min < 5) {
                _min = 5;
            }
            this.min_jpeg_quality = _min;
        }
    }
    increase_quality() {
        this.set_max_quality(this.max_jpeg_quality + 5);
    }
    decrease_quality() {
        this.set_max_quality(this.max_jpeg_quality - 5);
    }
    load() {
        var _quality_max = window.localStorage.getItem('mjpeg.quality.max');
        if (_quality_max != null) {
            this.set_max_quality(JSON.parse(_quality_max));
        }
    }
    save() {
        window.localStorage.setItem('mjpeg.quality.max', JSON.stringify(this.max_jpeg_quality));
    }
}


class MJPEGFrameController {
    constructor() {
        this.target_fps = 16;
        this.max_jpeg_quality = 50;
        this.display_resolution = 'default';
        this.jpeg_quality = 20;
        this.min_jpeg_quality = 25;
    }
    update_quality(actual_fps) {
        var q_step = Math.min(1, Math.max(-1, actual_fps - this.target_fps));
        this.jpeg_quality = Math.min(this.max_jpeg_quality, Math.max(this.min_jpeg_quality, this.jpeg_quality + q_step));
    }
}


// One for each camera.
var front_camera_frame_controller = new MJPEGFrameController();
var rear_camera_frame_controller = new MJPEGFrameController();

// Accessed outside of this module.
var mjpeg_page_controller = {
    store: new MJPEGControlLocalStorage(),

    init: function(frame_controllers) {
        this.frame_controllers = frame_controllers;
        this.apply_limits();
    },
    apply_limits: function() {
        _instance = this;
        this.frame_controllers.forEach(function(fc) {
            fc.max_jpeg_quality = _instance.get_max_quality();
            fc.min_jpeg_quality = _instance.get_min_quality();
        });
    },
    get_max_quality: function() {
        return this.store.max_jpeg_quality;
    },
    get_min_quality: function() {
        return this.store.min_jpeg_quality;
    },
    increase_quality: function() {
        this.store.increase_quality();
        this.apply_limits();
        this.store.save();
    },
    decrease_quality: function() {
        this.store.decrease_quality();
        this.apply_limits();
        this.store.save();
    }
}
mjpeg_page_controller.init([front_camera_frame_controller, rear_camera_frame_controller]);


if (page_utils.get_stream_type() == 'mjpeg') {
    class CameraController {
        constructor(camera_position, el_main_image, el_preview_image, frame_controller) {
            this.camera_position = camera_position;
            this.main_image = el_main_image;
            this.preview_image = el_preview_image;
            this.frame_controller = frame_controller;
            this.request_start = performance.now();
            this.request_time = 0;
            this.request_timeout = 0;
            // larger = more smoothing
            this.request_time_smoothing = 0.01;
            this.request_target_timeout = 1000. / frame_controller.target_fps;
            this.socket_close_timer_id = null;
            this.socket = null;
        }
        clear_socket_timeout() {
            if (this.socket_close_timer_id != undefined) {
                clearTimeout(this.socket_close_timer_id);
            }
        }
        update_framerate() {
            var end_time = performance.now();
            var duration = end_time - this.request_start;
            this.request_start = end_time;
            // smooth with moving average
            this.request_time = (this.request_time * this.request_time_smoothing) + (duration * (1.0 - this.request_time_smoothing));
            this.request_timeout = Math.max(0, this.request_target_timeout - this.request_time);
            var actual_fps = Math.round(1000 / this.request_time);
            return actual_fps;
        }
        socket_close(socket) {
            socket.close(4001, "Done waiting for the server to respond.");
            this.update_framerate();
        }
        capture(socket) {
            var _instance = this;
            this.clear_socket_timeout();
            this.socket_close_timer_id = setTimeout(function() {_instance.socket_close(socket);}, 1000);
            // E.g. '{"camera": "front", "quality": 50, "display": "vga"}'
            if (socket != undefined && socket.readyState == 1) {
                socket.send(JSON.stringify({
                    camera: _instance.camera_position,
                    quality: _instance.frame_controller.jpeg_quality,
                    display: _instance.frame_controller.display_resolution
                }));
            }
        }
        start_socket() {
            var _instance = this;
            socket_utils.create_socket("/ws/cam", true, 100, function(ws) {
                _instance.socket = ws;
                ws.attempt_reconnect = true;
                ws.is_reconnect = function() {
                    return ws.attempt_reconnect;
                }
                ws.onopen = function() {
                    console.log("MJPEG " + _instance.camera_position + " camera connection established.");
                    _instance.capture(ws);
                };
                ws.onclose = function() {
                    console.log("MJPEG " + _instance.camera_position + " camera connection closed.");
                };
                ws.onmessage = function(evt) {
                    _instance.clear_socket_timeout();
                    var _fps =_instance.update_framerate();
                    var _blob = window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"}));
                    _instance.frame_controller.update_quality(_fps);
                    _instance.main_image.src = _blob;
                    _instance.preview_image.src = _blob;
                    setTimeout(function() {_instance.capture(ws);}, _instance.request_timeout);
                };
            });
        }
        stop_socket() {
            if (this.socket != undefined) {
                this.socket.attempt_reconnect = false;
                if (this.socket.readyState < 2) {
                    this.socket.close();
                }
            }
            this.socket = null;
        }
    }

    // Setup once.
    document.addEventListener("DOMContentLoaded", function() {
        front_camera_main = document.createElement("img");
        front_camera_main.id = 'mjpeg_front_camera_main_image';
        front_camera_preview = document.getElementById('camera0_preview');
        document.getElementById('cameras_container').appendChild(front_camera_main);
        front_camera = new CameraController('front', front_camera_main, front_camera_preview, front_camera_frame_controller);

        rear_camera_container = document.createElement("div");
        rear_camera_container.id = 'mjpeg_rear_camera_main_container';
        document.body.appendChild(rear_camera_container);

        rear_camera_main = document.createElement("img");
        rear_camera_main.id = 'mjpeg_rear_camera_main_image';
        rear_camera_container.appendChild(rear_camera_main);
        rear_camera_preview = document.getElementById('camera1_preview');
        rear_camera = new CameraController('rear', rear_camera_main, rear_camera_preview, rear_camera_frame_controller);

        $('div#mjpeg_rear_camera_main_container').draggable();
        $('img#mjpeg_rear_camera_main_image').resizable();
    });
}

function mjpeg_start_all() {
    if (page_utils.get_stream_type() == 'mjpeg' && front_camera != undefined && front_camera.socket == undefined) {
        front_camera.start_socket();
        rear_camera.start_socket();
    }
}

function mjpeg_stop_all() {
    if (page_utils.get_stream_type() == 'mjpeg' && front_camera != undefined && front_camera.socket != undefined) {
        front_camera.stop_socket();
        rear_camera.stop_socket();
    }
}
