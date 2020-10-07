
window.addEventListener("gamepadconnected", function(e) { gamepad_controller.connect(e, true); }, false);
window.addEventListener("gamepaddisconnected", function(e) { gamepad_controller.connect(e, false); }, false);

var teleop_screen = {
    is_connection_ok: 0,
    controller_status: 0,
    c_msg_connection_lost: "Connection lost - please wait or refresh the page.",
    c_msg_controller_err: "Controller not detected - please press a button on the device.",
    c_msg_teleop_view_only: "Another user is in control - please remain as viewer or refresh the page to attempt control.",
    selectable_cameras: ['none', 'front', 'rear'],
    selected_camera: 'none',
    camera_selection_listeners: [],
    camera_cycle_timer: null,

    add_camera_selection_listener: function(cb) {
        this.camera_selection_listeners.push(cb);
    },

    notify_camera_selection_listeners: function(current) {
        this.camera_selection_listeners.forEach(function(cb) {
            cb(current);
        });
    },

    is_camera_selected: function(name) {
        return this.selected_camera == name;
    },

    on_hide_rear_camera: function() {
        if (this.selectable_cameras.indexOf('rear') != -1) {
            this.selectable_cameras.pop();
        }
        if (this.is_camera_selected('rear')) {
            this.select_camera('none');
        }
    },

    on_show_rear_camera: function() {
        if (this.selectable_cameras.indexOf('rear') == -1) {
            this.selectable_cameras.push('rear');
        }
    },

    select_camera: function(name) {
        this.selected_camera = name;
        this.notify_camera_selection_listeners(this.selected_camera);
    },

    request_camera_cycle: function(direction) {
        if (this.camera_cycle_timer == undefined) {
            this.camera_cycle_timer = setTimeout(function() {teleop_screen.cycle_camera_selection(direction);}, 250);
        }
    },

    cycle_camera_selection: function(direction) {
        if (direction == 'clockwise') {
            idx = this.selectable_cameras.indexOf(this.selected_camera) + 1;
            if (idx >= this.selectable_cameras.length) {
                idx = 0;
            }
            this.select_camera(this.selectable_cameras[idx]);
        } else {
            idx = this.selectable_cameras.indexOf(this.selected_camera) - 1;
            if (idx < 0) {
                idx = this.selectable_cameras.length - 1;
            }
            this.select_camera(this.selectable_cameras[idx]);
        }
        this.camera_cycle_timer = null;
    },

    update: function(command) {
        is_connection_ok = teleop_screen.is_connection_ok;
        controller_status = teleop_screen.controller_status;
        c_msg_connection_lost = teleop_screen.c_msg_connection_lost;
        c_msg_controller_err = teleop_screen.c_msg_controller_err;
        c_msg_teleop_view_only = teleop_screen.c_msg_teleop_view_only;
        var message_box = $('div#message_box');
        if (!is_connection_ok) {
            message_box.text(c_msg_connection_lost);
            message_box.removeClass();
            message_box.addClass('message error_message');
            message_box.show();
        } else if (controller_status == 0) {
            message_box.text(c_msg_controller_err);
            message_box.removeClass();
            message_box.addClass('message warning_message');
            message_box.show();
        } else if (controller_status == 2) {
            message_box.text(c_msg_teleop_view_only);
            message_box.removeClass();
            message_box.addClass('message warning_message');
            message_box.show();
        } else {
            message_box.hide();
        }
        //
        if (command.arrow_left) {
            this.request_camera_cycle('counter_clockwise');
        } else if (command.arrow_right) {
            this.request_camera_cycle('clockwise');
        }
    }
}

gamepad_controller.capture = function(cs_response) {
    gc = gamepad_controller;
    ct = gc.controller;
    command = {};
    gc_active = ct.poll();
    if (gc_active) {
        // Skip buttons when not pressed to save bandwidth.
        command.steering = ct.steering;
        command.throttle = ct.throttle;
        command.pan = ct.pan;
        command.tilt = ct.tilt;
        command.camera_id = -1;
        if (teleop_screen.selected_camera == 'front') {
            command.camera_id = 0;
        } else if (teleop_screen.selected_camera == 'rear') {
            command.camera_id = 1;
        }
        if (ct.button_center) {
            command.button_center = ct.button_center;
        }
        if (ct.button_left) {
            command.button_left = ct.button_left;
        }
        if (ct.button_right) {
            command.button_right = ct.button_right;
        }
        if (ct.button_a) {
            command.button_a = ct.button_a;
        }
        if (ct.button_b) {
            command.button_b = ct.button_b;
        }
        if (ct.button_x) {
            command.button_x = ct.button_x;
        }
        if (ct.button_y) {
            command.button_y = ct.button_y;
        }
        if (ct.arrow_up) {
            command.arrow_up = ct.arrow_up;
        }
        if (ct.arrow_down) {
            command.arrow_down = ct.arrow_down;
        }
        if (ct.arrow_left) {
            command.arrow_left = ct.arrow_left;
        }
        if (ct.arrow_right) {
            command.arrow_right = ct.arrow_right;
        }
    }
    if (gc.socket != undefined && gc.socket.readyState == 1) {
        gc.socket.send(JSON.stringify(command));
    }
    if (cs_response != undefined && cs_response.control == 'operator') {
        teleop_screen.controller_status = gc_active;
    } else if (cs_response != undefined) {
        teleop_screen.controller_status = 2;
    }
    teleop_screen.update(command);
}
gamepad_controller.start_socket = function() {
    socket_utils.create_socket("/ws/ctl", false, 100, function(ws) {
        gamepad_controller.socket = ws;
        ws.attempt_reconnect = true;
        ws.is_reconnect = function() {
            return ws.attempt_reconnect;
        }
        ws.onopen = function() {
            console.log("Operator socket connection was established.");
            teleop_screen.is_connection_ok = 1;
            gamepad_controller.capture();
        };
        ws.onclose = function() {
            teleop_screen.is_connection_ok = 0;
            teleop_screen.update({});
            console.log("Operator socket connection was closed.");
        };
        ws.onerror = function() {
            teleop_screen.controller_status = gamepad_controller.controller.poll();
            teleop_screen.is_connection_ok = 0;
            teleop_screen.update({});
        };
        ws.onmessage = function(evt) {
            var message = JSON.parse(evt.data);
            setTimeout(function() {gamepad_controller.capture(message);}, 0);
        };
    });
}
gamepad_controller.stop_socket = function() {
    gamepad_controller.socket.attempt_reconnect = false;
    if (gamepad_controller.socket.readyState < 2) {
        gamepad_controller.socket.close();
    }
    gamepad_controller.socket = null;
}

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
    if (log_controller.socket != undefined && log_controller.socket.readyState == 1) {
        log_controller.socket.send('{}');
    }
}
log_controller.start_socket = function() {
    socket_utils.create_socket("/ws/log", false, 100, function(ws) {
        log_controller.socket = ws;
        ws.attempt_reconnect = true;
        ws.is_reconnect = function() {
            return ws.attempt_reconnect;
        }
        ws.onopen = function() {
            console.log("Logger socket connection was established.");
            log_controller.capture();
        };
        ws.onclose = function() {
            console.log("Logger socket connection was closed.");
        };
        ws.onmessage = function(evt) {
            view = log_controller;
            var el_pilot_steering = $('span#pilot_steering');
            var el_pilot_throttle = $('span#pilot_throttle');
            var el_inference_penalty = $('span#inference_penalty');
            var el_inference_obstacle = $('span#inference_obstacle');
            var el_inference_fps = $('span#inference_fps');
            var el_alpha_speed = $('div#alpha_speed_value');
            var el_alpha_speed_label = $('div#alpha_speed_label');
            var el_beta_speed_container = $('div#beta_speed');
            var el_beta_speed = $('div#beta_speed_value');
            var el_steering_wheel = $('img#steeringWheel');
            var el_turn_arrow = $('img#arrow');
            var el_autopilot_status = $('#autopilot_status');
            //
            var command = JSON.parse(evt.data);
            el_pilot_steering.text(command.ste.toFixed(3));
            el_pilot_throttle.text(command.thr.toFixed(3));
            el_inference_penalty.text(command.debug3.toFixed(2));
            el_inference_obstacle.text(command.debug2.toFixed(2));
            el_inference_fps.text(command.debug7.toFixed(0));
            // el_state_recorder.text(view.str_recorder(command.rec_mod, command.rec_act));
            // speed is the desired speed
            // vel_y is the actual vehicle speed
            // console.log(command);
            var is_on_autopilot = command.ctl == 5;
            if (is_on_autopilot) {
                el_alpha_speed.text(command.max_speed.toFixed(1));
                el_beta_speed.text(command.speed.toFixed(1));
            } else {
                el_alpha_speed.text(command.vel_y.toFixed(1));
            }
            //
            if (view.command_turn != command.turn) {
                view.command_turn = command.turn;
                switch(command.turn) {
                    case "intersection.left":
                        el_turn_arrow.attr('src', 'im_arrow_left.png?v=0.20.6');
                        break;
                    case "intersection.right":
                        el_turn_arrow.attr('src', 'im_arrow_right.png?v=0.20.6');
                        break;
                    case "intersection.ahead":
                        el_turn_arrow.attr('src', 'im_arrow_up.png?v=0.20.6');
                        break;
                    default:
                        el_turn_arrow.attr('src', 'im_arrow_none.png?v=0.20.6');
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
                    el_steering_wheel.attr('src', 'im_wheel_blue.png?v=0.20.6');
                } else if (can_continue) {
                    el_steering_wheel.attr('src', 'im_wheel_black.png?v=0.20.6');
                } else {
                    el_steering_wheel.attr('src', 'im_wheel_red.png?v=0.20.6');
                }
                if (is_on_autopilot) {
                    el_alpha_speed_label.text('MAX');
                    el_beta_speed_container.show();
                    el_autopilot_status.text('AUTO');
                } else {
                    el_alpha_speed_label.text('km/h');
                    el_beta_speed_container.hide();
                    el_autopilot_status.text('TELE');
                }
            }
            var display_rotation = Math.floor(command.ste * 90.0)
            el_steering_wheel.css('transform', "rotate(" + display_rotation + "deg)");
            //
            setTimeout(log_controller.capture, 40);
        };
    });
}
log_controller.stop_socket = function() {
    log_controller.socket.attempt_reconnect = false;
    if (log_controller.socket.readyState < 2) {
        log_controller.socket.close();
    }
    log_controller.socket = null;
}

function teleop_start_all() {
    if (gamepad_controller.socket == undefined) {
        gamepad_controller.start_socket();
    }
    if (log_controller.socket == undefined) {
        log_controller.start_socket();
    }
}

function teleop_stop_all() {
    if (gamepad_controller.socket != undefined) {
        gamepad_controller.stop_socket();
    }
    if (log_controller.socket != undefined) {
        log_controller.stop_socket();
    }
}
