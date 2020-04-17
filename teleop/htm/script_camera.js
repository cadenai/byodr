
var camera_controller = {
    actual_fps: 0,
    target_fps: 16,
    display_resolution: 'default',
    jpeg_quality: 50,
    min_jpeg_quality: 25,
    max_jpeg_quality: 95,
    request_start: performance.now(),
    request_time: 0,
    request_timeout: 0,
    // larger = more smoothing
    request_time_smoothing: 0.5,
    request_target_timeout: 100,
    socket_close_timer_id: null,

    init: function(doc_location) {
        var url_params = new URL(doc_location).searchParams;
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

camera_controller.init(document.location);
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
        $('img#liveImg').attr("src", window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"})));
        $('span#frame_fps').text(camera_controller.actual_fps);
        $('span#frame_quality').text(camera_controller.jpeg_quality);
        setTimeout(function() {camera_controller.capture(ws);}, camera_controller.request_timeout);
    };
});
//socket_utils.create_socket("/ws/cam", true, 100, function(ws) {
//    ws.onopen = function() {
//        console.log("Camera socket connection two established.");
//        camera_controller.capture(ws);
//    };
//    ws.onmessage = function(evt) {
//        camera_controller.clear_socket_timeout();
//        $('img#liveImg').attr("src", window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"})));
//        setTimeout(function() {camera_controller.capture(ws);}, camera_controller.request_timeout);
//    };
//});
