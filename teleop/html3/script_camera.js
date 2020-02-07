
var camera_controller = {
    actual_fps: 0,
    target_fps: 15,
    display_resolution: 'default',
    jpeg_quality: 30,
    min_jpeg_quality: 30,
    max_jpeg_quality: 95,
    request_start: performance.now(),
    request_time: 0,
    request_timeout: 0,
    // larger = more smoothing
    request_time_smoothing: 0.8,
    request_target_timeout: 100,

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
camera_controller.socket = socket_utils.create_socket("/ws/cam", true);
camera_controller.capture = function() {
    cc = camera_controller;
    // E.g. '{"quality": 50, "display": "vga"}'
    cc.socket.send(JSON.stringify({
        quality: cc.jpeg_quality,
        display: cc.display_resolution
    }));
}
camera_controller.socket.onopen = function() {
    console.log("The camera socket connection was established.");
    camera_controller.capture();
};
camera_controller.socket.onmessage = function(evt) {
    camera_controller.update_framerate();
    camera_controller.update_quality();
    $('img#liveImg').attr("src", window.URL.createObjectURL(new Blob([new Uint8Array(evt.data)], {type: "image/jpeg"})));
    $('span#frame_fps').text(camera_controller.actual_fps);
    $('span#frame_quality').text(camera_controller.jpeg_quality);
    setTimeout(camera_controller.capture, camera_controller.request_timeout);
};
