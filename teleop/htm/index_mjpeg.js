class MJPEGFrameController {
    constructor() {
        this.actual_fps = 0;
        this.target_fps = 1;
        this.max_jpeg_quality = 50;
        this.display_resolution = 'WQVGA';
        this.jpeg_quality = 20;
        this.min_jpeg_quality = 25;
        this.request_start = performance.now();
        this.request_time = 0;
        this.request_timeout = 0;
        // larger = more smoothing
        this.request_time_smoothing = 0.20;
        this.request_target_timeout = 0;
        this.set_slow();
    }
    set_target_fps(v) {
        this.target_fps = v;
        this.request_target_timeout = 1000. / this.target_fps;
    }
    set_slow() {
        this.set_target_fps(2);
    }
    set_fast() {
        this.set_target_fps(16);
    }
    update_framerate() {
        var end_time = performance.now();
        var duration = end_time - this.request_start;
        this.request_start = end_time;
        // smooth with moving average
        this.request_time = (this.request_time * this.request_time_smoothing) + (duration * (1.0 - this.request_time_smoothing));
        this.request_timeout = Math.max(0, this.request_target_timeout - this.request_time);
        var actual_fps = Math.round(1000 / this.request_time);
        var q_step = Math.min(1, Math.max(-1, actual_fps - this.target_fps));
        this.jpeg_quality = Math.min(this.max_jpeg_quality, Math.max(this.min_jpeg_quality, this.jpeg_quality + q_step));
        this.actual_fps = actual_fps;
    }
}

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

class CameraController {
    constructor(camera_position, frame_controller, message_callback) {
        this.camera_position = camera_position;
        this.message_callback = message_callback;
        this.frame_controller = frame_controller;
        this.socket_close_timer_id = null;
        this.socket = null;
    }
    clear_socket_timeout() {
        if (this.socket_close_timer_id != undefined) {
            clearTimeout(this.socket_close_timer_id);
        }
    }
    socket_close(socket) {
        socket.close(4001, "Done waiting for the server to respond.");
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
                _instance.frame_controller.update_framerate();
                _instance.message_callback(evt.data);
                setTimeout(function() {_instance.capture(ws);}, _instance.frame_controller.request_timeout);
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


// One for each camera.
var front_camera_frame_controller = new MJPEGFrameController();
var rear_camera_frame_controller = new MJPEGFrameController();

// Accessed outside of this module.
var mjpeg_page_controller = {
    store: new MJPEGControlLocalStorage(),
    camera_image_listeners: [],

    init: function(frame_controllers) {
        this.frame_controllers = frame_controllers;
        this.apply_limits();
    },
    add_camera_image_listener: function(cb) {
        this.camera_image_listeners.push(cb);
    },
    notify_camera_image_listeners: function(camera_position, _blob) {
        this.camera_image_listeners.forEach(function(cb) {
            cb(camera_position, _blob);
        });
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
    },
    refresh_page_values: function() {
        $('span#mjpeg_quality_val').text(this.get_max_quality());
    },
    caret_up: function() {
        this.increase_quality();
        this.refresh_page_values();
    },
    caret_down: function() {
        this.decrease_quality();
        this.refresh_page_values();
    }
}
mjpeg_page_controller.init([front_camera_frame_controller, rear_camera_frame_controller]);

// Setup both the camera socket consumers.
mjpeg_rear_camera = new CameraController('rear', rear_camera_frame_controller, function(im_data) {
    var _blob = window.URL.createObjectURL(new Blob([new Uint8Array(im_data)], {type: "image/jpeg"}));
    mjpeg_page_controller.notify_camera_image_listeners(mjpeg_rear_camera.camera_position, _blob);
    $('span#rear_camera_framerate').text(rear_camera_frame_controller.actual_fps.toFixed(0));
});
mjpeg_front_camera = new CameraController('front', front_camera_frame_controller, function(im_data) {
    var _blob = window.URL.createObjectURL(new Blob([new Uint8Array(im_data)], {type: "image/jpeg"}));
    mjpeg_page_controller.notify_camera_image_listeners(mjpeg_front_camera.camera_position, _blob);
    $('span#front_camera_framerate').text(front_camera_frame_controller.actual_fps.toFixed(0));
});



document.addEventListener("DOMContentLoaded", function() {
    const el_rear_preview_image = $('img#mjpeg_rear_camera_preview_image');
    const el_front_preview_image = $('img#mjpeg_front_camera_preview_image');
    el_rear_preview_image.click(function() {
        el_front_preview_image.removeClass('active');
        el_rear_preview_image.addClass('active');
        teleop_screen.activate_camera('rear');
    });
    el_front_preview_image.click(function() {
        el_rear_preview_image.removeClass('active');
        el_front_preview_image.addClass('active');
        teleop_screen.activate_camera('front');
    });
    mjpeg_page_controller.add_camera_image_listener(function(position, _blob) {
        if (position == 'front') {
            el_front_preview_image.attr('src', _blob);
        } else {
            el_rear_preview_image.attr('src', _blob);
        }
    });
    // Build the main view if required.
    if (page_utils.get_stream_type() == 'mjpeg') {
        // The main viewport is also mjpeg.
        const el_main_camera_image = document.createElement("img");
        el_main_camera_image.style.cssText = 'width: 100% !important; height: 100% !important;';
        document.getElementById('viewport_container').appendChild(el_main_camera_image);
        // Render images for the active camera.
        mjpeg_page_controller.add_camera_image_listener(function(position, _blob) {
            if (teleop_screen.active_camera == position) {
                el_main_camera_image.src = _blob;
            }
        });
        // Set the socket desired fps when the active camera changes.
        teleop_screen.add_camera_activation_listener(function(position) {
            if (position == 'front') {
                rear_camera_frame_controller.set_slow();
                front_camera_frame_controller.set_fast();
            } else {
                front_camera_frame_controller.set_slow();
                rear_camera_frame_controller.set_fast();
            }
        });
        front_camera_frame_controller.set_fast();
    }
});

function mjpeg_start_all() {
    if (mjpeg_rear_camera != undefined && mjpeg_rear_camera.socket == undefined) {
        mjpeg_rear_camera.start_socket();
    }
    if (mjpeg_front_camera != undefined && mjpeg_front_camera.socket == undefined) {
        mjpeg_front_camera.start_socket();
    }
}

function mjpeg_stop_all() {
    if (mjpeg_rear_camera != undefined && mjpeg_rear_camera.socket != undefined) {
        mjpeg_rear_camera.stop_socket();
    }
    if (mjpeg_front_camera != undefined && mjpeg_front_camera.socket != undefined) {
        mjpeg_front_camera.stop_socket();
    }
}
