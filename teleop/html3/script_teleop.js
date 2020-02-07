
window.addEventListener("gamepadconnected", function(e) { gamepad_controller.connect(e, true); }, false);
window.addEventListener("gamepaddisconnected", function(e) { gamepad_controller.connect(e, false); }, false);

gamepad_controller.socket = socket_utils.create_socket("/ws/ctl", false);
gamepad_controller.capture = function() {
    gc = gamepad_controller;
    ct = gc.controller;
    // Skip button message content when not pressed to save bandwidth.
    msg = {steering: ct.steering, throttle: ct.throttle};
    if (ct.button_center) {
        msg.button_back = ct.button_center;
    }
    if (ct.button_left) {
        msg.button_left = ct.button_left;
    }
    if (ct.button_right) {
        msg.button_right = ct.button_right;
    }
    if (ct.button_a) {
        msg.button_a = ct.button_a;
    }
    if (ct.button_b) {
        msg.button_b = ct.button_b;
    }
    if (ct.button_x) {
        msg.button_x = ct.button_x;
    }
    if (ct.button_y) {
        msg.button_y = ct.button_y;
    }
    if (ct.arrow_up) {
        msg.arrow_up = ct.arrow_up;
    }
    if (ct.arrow_down) {
        msg.arrow_down = ct.arrow_down;
    }
    gc.socket.send(JSON.stringify(msg));
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
var log_controller = {
    command_turn: null,
    command_ctl: null,
    recorders: {551: 'driving', 594: 'interventions'},

    str_recorder: function(mode, active) {
        mode = parseInt(mode);
        active = Boolean(active);
        str_mode = (mode in this.recorders) ? this.recorders[mode] : 'n/a'
        return str_mode + ' ' + (active ? '[ON]' : '(off)');
    }
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
    var el_inference_corridor = $('span#inference_corridor');
    var el_inference_obstacle = $('span#inference_obstacle');
    var el_state_recorder = $('span#state_recorder');
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
    el_inference_corridor.text(command.debug1.toFixed(3));
    el_inference_obstacle.text(command.debug2.toFixed(3));
    el_state_recorder.text(view.str_recorder(command.rec_mod, command.rec_act));
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
