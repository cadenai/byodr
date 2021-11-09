class MJPEGFrameController {
    constructor() {
        this.actual_fps = 0;
        this.target_fps = 1;
        this.max_jpeg_quality = 50;
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
                quality: _instance.frame_controller.jpeg_quality
            }));
        }
    }
    start_socket() {
        var _instance = this;
        var _cam_uri = "/ws/cam/" + _instance.camera_position;
        socket_utils.create_socket(_cam_uri, true, 100, function(ws) {
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
                setTimeout(function() {_instance.message_callback(evt.data);}, 0);
                setTimeout(function() {_instance.capture(ws);}, _instance.frame_controller.request_timeout);
            };
        });
    }
    stop_socket() {
        if (this.socket != undefined) {
            this.socket.attempt_reconnect = false;
            if (this.socket.readyState < 2) {
                try {
                    this.socket.close();
                } catch(err) {}
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
    camera_init_listeners: [],
    camera_image_listeners: [],

    init: function(frame_controllers) {
        this.frame_controllers = frame_controllers;
        this.apply_limits();
    },
    add_camera_listener: function(cb_init, cb_image) {
        this.camera_init_listeners.push(cb_init);
        this.camera_image_listeners.push(cb_image);
    },
    notify_camera_init_listeners: function(camera_position, _cmd) {
        this.camera_init_listeners.forEach(function(cb) {
            cb(camera_position, _cmd);
        });
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
mjpeg_rear_camera = new CameraController('rear', rear_camera_frame_controller, function(evt_data) {
    if(typeof evt_data == "string") {
        cmd = JSON.parse(evt_data);
        mjpeg_page_controller.notify_camera_init_listeners(mjpeg_rear_camera.camera_position, cmd);
    } else {
        var _blob = window.URL.createObjectURL(new Blob([new Uint8Array(evt_data)], {type: "image/jpeg"}));
        mjpeg_page_controller.notify_camera_image_listeners(mjpeg_rear_camera.camera_position, _blob);
        $('span#rear_camera_framerate').text(rear_camera_frame_controller.actual_fps.toFixed(0));
    }
});

mjpeg_front_camera = new CameraController('front', front_camera_frame_controller, function(evt_data) {
    if(typeof evt_data == "string") {
        cmd = JSON.parse(evt_data);
        mjpeg_page_controller.notify_camera_init_listeners(mjpeg_front_camera.camera_position, cmd);
    } else {
        var _blob = window.URL.createObjectURL(new Blob([new Uint8Array(evt_data)], {type: "image/jpeg"}));
        mjpeg_page_controller.notify_camera_image_listeners(mjpeg_front_camera.camera_position, _blob);
        $('span#front_camera_framerate').text(front_camera_frame_controller.actual_fps.toFixed(0));
    }
});



document.addEventListener("DOMContentLoaded", function() {
    const el_rear_preview_image = $('img#mjpeg_rear_camera_preview_image');
    const el_front_preview_image = $('img#mjpeg_front_camera_preview_image');
    el_rear_preview_image.click(function() {
        el_front_preview_image.removeClass('active');
        el_rear_preview_image.addClass('active');
        teleop_screen.activate_camera('rear');
        page_utils.toggle_debug_values(false);
    });
    el_front_preview_image.click(function() {
        el_rear_preview_image.removeClass('active');
        el_front_preview_image.addClass('active');
        teleop_screen.activate_camera('front');
        page_utils.toggle_debug_values(true);
    });
    mjpeg_page_controller.add_camera_listener(
        function(position, _cmd) {},
        function(position, _blob) {
            if (position == 'front') {
                el_front_preview_image.attr('src', _blob);
            } else {
                el_rear_preview_image.attr('src', _blob);
            }
        }
    );
    // Build the main view if required.
    if (page_utils.get_stream_type() == 'mjpeg') {
        // const el_viewport = document.getElementById('viewport_container');
        // const el_main_camera_display = document.createElement("canvas");
        const el_main_camera_display = document.getElementById("viewport_canvas");
        // el_main_camera_display.style.cssText = 'width: 100% !important; height: 100% !important;';
        // The canvas dimension will be set when we open the websocket.
        // el_main_camera_display.width = 640;
        // el_main_camera_display.height = 480;
        // el_viewport.appendChild(el_main_camera_display);
        const display_ctx = el_main_camera_display.getContext('2d');
        // Render images for the active camera.
        mjpeg_page_controller.add_camera_listener(
            function(position, _cmd) {
                if (teleop_screen.active_camera == position && _cmd.action == "init") {
                    console.log("Display " + position + " received command ", _cmd);
                    el_main_camera_display.width = _cmd.width;
                    el_main_camera_display.height = _cmd.height;
                }
            },
            function(position, _blob) {
                if (teleop_screen.active_camera == position) {
                    var img = new Image();
                    img.onload = function() {
                        // Do not run the canvas draws in parallel.
                        display_ctx.drawImage(img, 0, 0);
                        teleop_screen.canvas_update(display_ctx);
                    };
                    img.src = _blob;
                }
            }
        );
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
