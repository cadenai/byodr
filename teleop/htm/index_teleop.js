
window.addEventListener("gamepadconnected", function(e) { gamepad_controller.connect(e, true); }, false);
window.addEventListener("gamepaddisconnected", function(e) { gamepad_controller.connect(e, false); }, false);

var teleop_screen = {
    is_connection_ok: 0,
    is_console_ok: 0,

    update: function() {
        is_connection_ok = teleop_screen.is_connection_ok;
        is_console_ok = teleop_screen.is_console_ok;
        var connection_error = $('div#connection_error');
        var console_warning = $('div#console_warning');
        if (is_connection_ok) {
            connection_error.hide();
        } else {
            connection_error.show();
        }
        if (is_console_ok) {
            console_warning.hide();
        } else {
            console_warning.show();
        }
    }
}

gamepad_controller.capture = function() {
    gc = gamepad_controller;
    ct = gc.controller;
    msg = {};
    gc_active = ct.poll();
    if (gc_active) {
        // Skip buttons when not pressed to save bandwidth.
        msg.steering = ct.steering;
        msg.throttle = ct.throttle;
        msg.pan = ct.pan;
        msg.tilt = ct.tilt;
        if (ct.button_center) {
            msg.button_center = ct.button_center;
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
    }
    gc.socket.send(JSON.stringify(msg));
    teleop_screen.is_console_ok = gc_active;
    teleop_screen.update();
}
socket_utils.create_socket("/ws/ctl", false, 100, function(ws) {
    gamepad_controller.socket = ws;
    ws.onopen = function() {
        console.log("The gamepad socket connection was established.");
        teleop_screen.is_connection_ok = 1;
        gamepad_controller.capture();
    };
    ws.onclose = function() {
        teleop_screen.is_connection_ok = 0;
        teleop_screen.update();
    };
    ws.onerror = function() {
        teleop_screen.is_console_ok = gamepad_controller.controller.poll();
        teleop_screen.is_connection_ok = 0;
        teleop_screen.update();
    };
    ws.onmessage = function() {
        setTimeout(gamepad_controller.capture, 0);
    };
});

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

log_controller.capture = function() {
    log_controller.socket.send('{}');
}
socket_utils.create_socket("/ws/log", false, 100, function(ws) {
    log_controller.socket = ws;
    ws.onopen = function() {
        console.log("The logger socket connection was established.");
        log_controller.capture();
    };
    ws.onmessage = function(evt) {
        view = log_controller;
        var el_pilot_steering = $('span#pilot_steering');
        var el_pilot_throttle = $('span#pilot_throttle');
        var el_inference_penalty = $('span#inference_penalty');
        var el_inference_surprise = $('span#inference_surprise');
        var el_inference_critic = $('span#inference_critic');
        var el_inference_fallback = $('span#inference_fallback');
        // var el_state_recorder = $('span#state_recorder');
        var el_max_speed_container = $('div#max_speed');
        var el_max_speed = $('div#max_speed_value');
        var el_current_speed = $('div#current_speed_value');
        var el_steering_wheel = $('img#steeringWheel');
        var el_turn_arrow = $('img#arrow');
        var el_autopilot_status = $('#autopilot_status');
        //
        var command = JSON.parse(evt.data);
        el_pilot_steering.text(command.ste.toFixed(2));
        el_pilot_throttle.text(command.thr.toFixed(2));
        el_inference_penalty.text(command.debug3.toFixed(2));
        el_inference_surprise.text(command.debug4.toFixed(2));
        el_inference_critic.text(command.debug7.toFixed(3));
        var txt_fallback = 'a';
        if (command.debug6 == 1) {
            txt_fallback = 'b';
        }
        el_inference_fallback.text(txt_fallback);
        // el_state_recorder.text(view.str_recorder(command.rec_mod, command.rec_act));
        // max_speed is the maximum speed
        // speed is the desired speed
        // vel_y is the actual vehicle speed
        // console.log(command);
        // Math.ceil(command.max_speed);
        var is_on_autopilot = command.ctl == 5;
        if (is_on_autopilot) {
            el_max_speed.text(command.max_speed.toFixed(1));
            el_current_speed.text(command.speed.toFixed(1));
        } else {
            el_current_speed.text(command.vel_y.toFixed(1));
        }
        //
        if (view.command_turn != command.turn) {
            view.command_turn = command.turn;
            switch(command.turn) {
                case "intersection.left":
                    el_turn_arrow.attr('src', 'im_arrow_left.png?v=0.11');
                    break;
                case "intersection.right":
                    el_turn_arrow.attr('src', 'im_arrow_right.png?v=0.11');
                    break;
                case "intersection.ahead":
                    el_turn_arrow.attr('src', 'im_arrow_up.png?v=0.11');
                    break;
                default:
                    el_turn_arrow.attr('src', '');
                    break;
            }
        }
        //
        var total_penalty = command.debug3;
        var can_continue = total_penalty < 1;
        var str_command_ctl = command.ctl + '_' + can_continue;
        if (view.command_ctl != str_command_ctl) {
            view.command_ctl = str_command_ctl;
            if (can_continue && is_on_autopilot) {
                el_steering_wheel.attr('src', 'im_wheel_blue.png?v=0.11');
            } else if (can_continue) {
                el_steering_wheel.attr('src', 'im_wheel_black.png?v=0.11');
            } else {
                el_steering_wheel.attr('src', 'im_wheel_red.png?v=0.11');
            }
            if (is_on_autopilot) {
                el_max_speed_container.show();
                el_autopilot_status.show();
            } else {
                el_max_speed_container.hide();
                el_autopilot_status.hide();
            }
        }
        var display_rotation = Math.floor(command.ste * 90.0)
        el_steering_wheel.css('transform', "rotate(" + display_rotation + "deg)");
        //
        setTimeout(log_controller.capture, 40);
    };
});