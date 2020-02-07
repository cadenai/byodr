
var GameController = {
    steering: 0,
    throttle: 0,
    button_y: 0,
    button_b: 0,
    button_x: 0,
    button_a: 0,
    button_left: 0,
    button_right: 0,
    button_center: 0,
    arrow_up: 0,
    arrow_down: 0,

    collapse: function(value, zone=0) {
        return Math.abs(value) <= zone ? 0 : value > 0 ? value - zone : value + zone;
    },

    poll: function() {}
}

var Xbox360Wired = extend(GameController, {
    gamepad: null,
    threshold: 0.18,
    scale: 0.5,

    poll: function() {
        pad = this.gamepad;
        this.steering = this.collapse(pad.axes[3], this.threshold) * this.scale;
        accelerate = pad.axes[2];
        brake = pad.axes[5];
        // Before activation of these axes their values are exactly zero.
        accelerate = accelerate == 0 ? 0 : (accelerate + 1) / 2;
        brake = brake == 0 ? 0 : (brake + 1) / 2;
        this.throttle = brake > 0.01 ? -1 * brake : accelerate;
        this.button_y = pad.buttons[14].value;
        this.button_b = pad.buttons[12].value;
        this.button_x = pad.buttons[13].value;
        this.button_a = pad.buttons[11].value;
        this.button_left = pad.buttons[8].value;
        this.button_right = pad.buttons[9].value;
        this.button_center = pad.buttons[10].value;
        this.arrow_up = pad.buttons[0].value;
        this.arrow_down = pad.buttons[1].value;
    }
});

var gamepad_controller = {
    controller: Object.create(GameController),

    connect: function(event, connecting) {
        var gamepad = event.gamepad;
        if (connecting) {
            if (gamepad.id == "45e-28e-Xbox 360 Wired Controller") {
                this.controller = Object.create(Xbox360Wired);
                this.controller.gamepad = gamepad;
                console.log("Connected " + gamepad.id + ".");
            } else {
                console.log("Gamepad " + gamepad.id + " not found.");
            }
        } else {
            this.controller = Object.create(GameController);
        }
    }
}

window.addEventListener("gamepadconnected", function(e) { gamepad_controller.connect(e, true); }, false);
window.addEventListener("gamepaddisconnected", function(e) { gamepad_controller.connect(e, false); }, false);

gamepad_controller.socket = socket_utils.create_socket("/ws/ctl", false);
gamepad_controller.capture = function() {
    gc = gamepad_controller;
    ct = gc.controller;
    gc.socket.send(JSON.stringify({
        steering: ct.steering,
        throttle: ct.throttle
    }));
    ct.poll();
}
gamepad_controller.socket.onopen = function() {
    console.log("The gamepad socket connection was established.");
    gamepad_controller.capture();
};
gamepad_controller.socket.onmessage = function(evt) {
    setTimeout(gamepad_controller.capture, 0);
};

//
//
//
var log_controller = {
}

log_controller.socket = socket_utils.create_socket("/ws/log", false);
log_controller.capture = function() {
    log_controller.socket.send('{}');
}
log_controller.socket.onopen = function() {
    console.log("The logger socket connection was established.");
    log_controller.capture();
};
log_controller.socket.onmessage = function(evt) {
    var log = JSON.parse(evt.data);
    $('span#pilot_steering').text(log['ste']);
    $('span#pilot_throttle').text(log['thr']);
    setTimeout(log_controller.capture, 40);
};
