var jpeg_control_params = new URL(document.location).searchParams;

if (jpeg_control_params.has('stream') && jpeg_control_params.get('stream') == 'jpeg') {
    var camera_controller = {
        actual_fps: 0,
        target_fps: 16,
        display_resolution: 'default',
        jpeg_quality: 50,
        min_jpeg_quality: 15,
        max_jpeg_quality: 75,
        request_start: performance.now(),
        request_time: 0,
        request_timeout: 0,
        // larger = more smoothing
        request_time_smoothing: 0.10,
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
            var url_params = new URL(document.location).searchParams;
            if (url_params.has('fps')) {
                this.target_fps = parseInt(url_params.get('fps'));
            }
            if (url_params.has('display')) {
                this.display_resolution = url_params.get('display');
            }
            if (url_params.has('quality')) {
                this.jpeg_quality = parseInt(url_params.get('quality'));
                this.min_jpeg_quality = this.jpeg_quality;
                this.max_jpeg_quality = this.jpeg_quality;
            }
            this.request_target_timeout = 1000. / this.target_fps
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
            this.actual_fps = Math.round(1000 / this.request_time);
        },

        update_quality: function() {
            var q_step = Math.min(1, Math.max(-1, this.actual_fps - this.target_fps));
            this.jpeg_quality = Math.min(this.max_jpeg_quality, Math.max(this.min_jpeg_quality, this.jpeg_quality + q_step));
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
            quality: camera_controller.jpeg_quality,
            display: camera_controller.display_resolution
        }));
    };
    socket_utils.create_socket("/ws/cam", true, 100, function(ws) {
        ws.onopen = function() {
            console.log("Camera socket connection established.");
            camera_controller.capture(ws);
        };
        ws.onmessage = function(evt) {
            camera_controller.clear_socket_timeout();
            camera_controller.update_framerate();
            camera_controller.update_quality();
            camera_controller.el_image.src = window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"}));
            $('span#frame_fps').text(camera_controller.actual_fps);
            $('span#frame_quality').text(camera_controller.jpeg_quality);
            setTimeout(function() {camera_controller.capture(ws);}, camera_controller.request_timeout);
        };
    });

    document.addEventListener("DOMContentLoaded", function() {
        camera_controller.init(document.getElementById('camera1'));
    });
}