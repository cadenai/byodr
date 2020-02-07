
var NoneController = {
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

    gamepad: function() {
        return navigator.getGamepads()[0];
    },

    poll: function() {}
}

var Xbox360StandardController = extend(NoneController, {
    threshold: 0.18,
    scale: 0.5,

    poll: function() {
        pad = this.gamepad();
        this.steering = this.collapse(pad.axes[2], this.threshold) * this.scale;
        accelerate = pad.buttons[6].value;
        brake = pad.buttons[7].value;
        this.throttle = brake > 0.01 ? -1 * brake : accelerate;
        this.button_y = pad.buttons[3].value;
        this.button_b = pad.buttons[1].value;
        this.button_x = pad.buttons[2].value;
        this.button_a = pad.buttons[0].value;
        this.button_left = pad.buttons[4].value;
        this.button_right = pad.buttons[5].value;
        this.button_center = pad.buttons[16].value;
        this.arrow_up = pad.buttons[12].value;
        this.arrow_down = pad.buttons[13].value;
    }
});

var PS4StandardController = extend(NoneController, {
    threshold: 0.05,
    scale: 0.5,

    poll: function() {
        pad = this.gamepad();
        this.steering = this.collapse(pad.axes[2], this.threshold) * this.scale;
        accelerate = pad.buttons[6].value;
        brake = pad.buttons[7].value;
        this.throttle = brake > 0.01 ? -1 * brake : accelerate;
        this.button_y = pad.buttons[3].value;
        this.button_b = pad.buttons[1].value;
        this.button_x = pad.buttons[2].value;
        this.button_a = pad.buttons[0].value;
        this.button_left = pad.buttons[4].value;
        this.button_right = pad.buttons[5].value;
        this.button_center = pad.buttons[17].value;
        this.arrow_up = pad.buttons[12].value;
        this.arrow_down = pad.buttons[13].value;
    }
});

var Xbox360Controller = extend(NoneController, {
    threshold: 0.18,
    scale: 0.5,

    poll: function() {
        pad = this.gamepad();
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
    controller: Object.create(NoneController),

    create_gamepad: function(gamepad) {
        // 45e-28e-Xbox 360 Wired Controller / Xbox Wireless Controller (STANDARD GAMEPAD Vendor: 045e Product: 02fd)
        // Wireless Controller (STANDARD GAMEPAD Vendor: 054c Product: 09cc)
        var gid = gamepad.id;
        var result = null;
        if (gamepad.mapping == 'standard') {
            if (gid.includes('45e')) {
                result = Object.create(Xbox360StandardController);
            } else if (gid.includes('54c')) {
                result = Object.create(PS4StandardController);
            }
        }
        return result;
    },

    connect: function(event, connecting) {
        var gamepad = event.gamepad;
        if (connecting) {
            controller = this.create_gamepad(gamepad);
            if (controller != undefined) {
                this.controller = controller;
                console.log("Connected " + gamepad.id + " - mapping = '" + gamepad.mapping + "'.");
            } else {
                console.log("Gamepad " + gamepad.id + " - mapping = '" + gamepad.mapping + "' not supported.");
            }
        } else {
            this.controller = Object.create(NoneController);
            console.log("Disconnected " + gamepad.id + ".");
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
        throttle: ct.throttle,
        button_back: ct.button_center,
        button_left: ct.button_left,
        button_right: ct.button_right,
        button_a: ct.button_a,
        button_b: ct.button_b,
        button_x: ct.button_x,
        button_y: ct.button_y,
        arrow_up: ct.arrow_up,
        arrow_down: ct.arrow_down
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
    command_turn: null,
    command_ctl: null
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
    view = log_controller;
    var el_pilot_steering = $('span#pilot_steering');
    var el_pilot_throttle = $('span#pilot_throttle');
    var el_frame_fps = $('span#frame_fps');
    var el_frame_quality = $('span#frame_quality');
    var el_current_speed = $('div#current_speed_value');
    var el_max_speed = $('div#desired_speed_value');
    var el_steering_wheel = $('img#steeringWheel');
    var el_turn_arrow = $('img#arrow');
    //
    var command = JSON.parse(evt.data);
    el_pilot_steering.text(command.ste.toFixed(3));
    el_pilot_throttle.text(command.thr.toFixed(3));
    // console.max_speed is the maximum speed
    // console.speed is the desired speed
    // console.vel_y is the actual vehicle speed
    // console.log(command);
    // Math.ceil(command.max_speed);
    el_current_speed.text(command.speed.toFixed(1));
    el_max_speed.text(command.max_speed.toFixed(1));
    if (view.command_turn != command.turn) {
        view.command_turn = command.turn;
        switch(command.turn) {
            case "intersection.left":
                el_turn_arrow.attr('src', 'im_arrow_left.png');
                break;
            case "intersection.right":
                el_turn_arrow.attr('src', 'im_arrow_right.png');
                break;
            default:
                el_turn_arrow.attr('src', 'im_arrow_up.png');
                break;
        }
    }
    //
    var steer_penalty = command.debug1 < 1;
    var str_command_ctl = command.ctl + '_' + steer_penalty;
    if (view.command_ctl != str_command_ctl) {
        view.command_ctl = str_command_ctl;
        el_steering_wheel.attr('src', 'im_wheel_a.png');
        if (command.ctl == 5) {
            if (steer_penalty) {
                el_steering_wheel.attr('src', 'im_wheel_b.png');
            } else {
                el_steering_wheel.attr('src', 'im_wheel_c.png');
            }
        }
    }
    var display_rotation = Math.floor(command.ste * 90.0)
    el_steering_wheel.css('transform', "rotate(" + display_rotation + "deg)");
    //
    setTimeout(log_controller.capture, 40);
};
