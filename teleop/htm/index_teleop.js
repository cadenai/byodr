
window.addEventListener("gamepadconnected", function(e) { gamepad_controller.connect(e, true); }, false);
window.addEventListener("gamepaddisconnected", function(e) { gamepad_controller.connect(e, false); }, false);

var teleop_screen = {
    is_connection_ok: 0,
    controller_status: 0,
    c_msg_connection_lost: "Connection lost - please wait or refresh the page.",
    c_msg_controller_err: "Controller not detected - please press a button on the device.",
    c_msg_teleop_view_only: "Another user is in control - please remain as viewer or refresh the page to attempt control.",
    active_camera: 'front',  // The active camera is rendered on the main display.
    camera_activation_listeners: [],
    selected_camera: null,  // Select a camera for ptz control.
    camera_selection_listeners: [],
    camera_cycle_timer: null,

    add_camera_activation_listener: function(cb) {
        this.camera_activation_listeners.push(cb);
    },

    activate_camera: function(name) {
        this.active_camera = name;
        this.select_camera(null);
        this.camera_activation_listeners.forEach(function(cb) {cb(name);});
    },

    add_camera_selection_listener: function(cb) {
        this.camera_selection_listeners.push(cb);
    },

    select_camera: function(name) {
        var is_selected = name == this.active_camera;
        this.selected_camera = name;
        this.camera_selection_listeners.forEach(function(cb) {cb(is_selected);});
    },

    cycle_camera_selection: function(direction) {
        if (this.selected_camera == this.active_camera) {
            this.select_camera(null);
        } else {
            this.select_camera(this.active_camera);
        }
        this.camera_cycle_timer = null;
    },

    request_camera_cycle: function(direction) {
        if (this.camera_cycle_timer == undefined) {
            this.camera_cycle_timer = setTimeout(function() {teleop_screen.cycle_camera_selection();}, 250);
        }
    },

    update: function(command) {
        is_connection_ok = teleop_screen.is_connection_ok;
        controller_status = teleop_screen.controller_status;
        c_msg_connection_lost = teleop_screen.c_msg_connection_lost;
        c_msg_controller_err = teleop_screen.c_msg_controller_err;
        c_msg_teleop_view_only = teleop_screen.c_msg_teleop_view_only;
        var show_message = false;
        var message_box = $('div#message_box');
        var viewport_container = $('div#viewport_container');
        if (!is_connection_ok) {
            message_box.text(c_msg_connection_lost);
            message_box.removeClass();
            message_box.addClass('message error_message');
            show_message = true;
        } else if (controller_status == 0) {
            message_box.text(c_msg_controller_err);
            message_box.removeClass();
            message_box.addClass('message warning_message');
            show_message = true;
        } else if (controller_status == 2) {
            message_box.text(c_msg_teleop_view_only);
            message_box.removeClass();
            message_box.addClass('message warning_message');
            show_message = true;
        }
        if (show_message) {
            viewport_container.height('calc(82vh - 0px)');
            message_box.show();
        } else {
            viewport_container.height('calc(90vh - 0px)');
            message_box.hide();
        }
        //
        if (command.arrow_left) {
            this.request_camera_cycle();
        } else if (command.arrow_right) {
            this.request_camera_cycle();
        }
        //
        if (command.button_left) {
            setTimeout(function() {viewport_container.fadeOut(100).fadeIn(100);}, 0);
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
    server_message_listeners: [],
    in_debug: 0,

    add_server_message_listener: function(cb) {
        this.server_message_listeners.push(cb);
    },

    notify_server_message_listeners: function(message) {
        this.server_message_listeners.forEach(function(cb) {
            cb(message);
        });
    },

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
            var el_inference_surprise = $('span#inference_surprise');
            var el_inference_critic = $('span#inference_critic');
            var el_inference_brake_critic = $('span#inference_brake_critic');
            var el_inference_obstacle = $('span#inference_obstacle');
            var el_inference_fps = $('span#inference_fps');
            var el_alpha_speed = $('div#alpha_speed_value');
            var el_alpha_speed_label = $('div#alpha_speed_label');
            var el_beta_speed_container = $('div#beta_speed');
            var el_beta_speed = $('div#beta_speed_value');
            var el_steering_wheel = $('img#steeringWheel');
            var el_turn_arrow = $('img#arrow');
            var el_autopilot_status = $('#autopilot_status');
            var el_navigation_geo_lat = $('span#navigation_geo_lat');
            var el_navigation_geo_long = $('span#navigation_geo_long');
            var el_navigation_heading = $('span#navigation_heading');
            var el_navigation_match_distance = $('span#navigation_match_image_distance');
            var el_navigation_current_command = $('span#navigation_current_command');
            var el_navigation_direction = $('span#navigation_direction');

            //
            var command = JSON.parse(evt.data);
            view.notify_server_message_listeners(command);
            el_pilot_steering.text(command.ste.toFixed(3));
            el_pilot_throttle.text(command.thr.toFixed(3));

            el_navigation_geo_lat.text(command.geo_lat.toFixed(6));
            el_navigation_geo_long.text(command.geo_long.toFixed(6));
            el_navigation_heading.text(command.geo_head.toFixed(2));

            const _debug = view.in_debug;
            // It may be the inference service is not yet available.
            if (command.inf_surprise != undefined) {
                el_inference_brake_critic.text(command.inf_brake_critic.toFixed(2));
                el_inference_obstacle.text(command.inf_brake.toFixed(2));
                el_inference_surprise.text(command.inf_surprise.toFixed(3));
                el_inference_critic.text(command.inf_critic.toFixed(3));
                el_inference_fps.text(command.inf_hz.toFixed(0));
                const column_id = _debug? 1 : 0;
                el_navigation_match_distance.text(command.nav_distance[column_id].toFixed(3));
                el_navigation_current_command.text(command.nav_command.toFixed(2));
                el_navigation_direction.text(command.nav_direction.toFixed(3));
            }
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
                        el_turn_arrow.attr('src', 'im_arrow_left.png?v=0.50.0');
                        break;
                    case "intersection.right":
                        el_turn_arrow.attr('src', 'im_arrow_right.png?v=0.50.0');
                        break;
                    case "intersection.ahead":
                        el_turn_arrow.attr('src', 'im_arrow_up.png?v=0.50.0');
                        break;
                    default:
                        el_turn_arrow.attr('src', 'im_arrow_none.png?v=0.50.0');
                        break;
                }
            }
            //
            var total_penalty = command.inf_penalty;
            var can_continue = total_penalty < 1;
            var str_command_ctl = command.ctl + '_' + can_continue;
            if (view.command_ctl != str_command_ctl) {
                view.command_ctl = str_command_ctl;
                if (can_continue && is_on_autopilot) {
                    el_steering_wheel.attr('src', 'im_wheel_blue.png?v=0.50.0');
                } else if (can_continue) {
                    el_steering_wheel.attr('src', 'im_wheel_black.png?v=0.50.0');
                } else {
                    el_steering_wheel.attr('src', 'im_wheel_red.png?v=0.50.0');
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

page_utils.add_toggle_debug_values_listener(function(collapse) {
    if (collapse) {
        log_controller.in_debug = 0;
        $("div#debug_drive_values").invisible();
        $("div#pilot_drive_values").css({'cursor': 'zoom-in'});
    } else {
        log_controller.in_debug = 1;
        $("div#debug_drive_values").visible();
        $("div#pilot_drive_values").css({'cursor': 'zoom-out'});
    }
});

function teleop_start_all() {
    gamepad_controller.reset();
    if (gamepad_controller.socket == undefined) {
        gamepad_controller.start_socket();
    }
    if (log_controller.socket == undefined) {
        log_controller.start_socket();
    }
}

function teleop_stop_all() {
    gamepad_controller.reset();
    if (gamepad_controller.socket != undefined) {
        gamepad_controller.stop_socket();
    }
    if (log_controller.socket != undefined) {
        log_controller.stop_socket();
    }
}
