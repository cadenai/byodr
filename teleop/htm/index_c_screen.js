
var screen_utils = {
    _version: '0.50.0',

    _decorate_server_message: function(message) {
        message._is_on_autopilot = message.ctl == 5;
        message._has_passage = message.inf_penalty < 1;
        return message;
    },

    _turn_arrow_url: function(turn) {
        switch(turn) {
            case "intersection.left":
                return 'im_arrow_left.png?v=' + this._version;
            case "intersection.right":
                return 'im_arrow_right.png?v=' + this._version;
            case "intersection.ahead":
                return 'im_arrow_up.png?v=' + this._version;
            default:
                return 'im_arrow_none.png?v=' + this._version;
        }
    },

    _steering_wheel_url: function(message) {
        if (message._is_on_autopilot && message._has_passage) {
            return 'im_wheel_blue.png?v=' + this._version;
        }
        if (message._has_passage) {
            return 'im_wheel_black.png?v=' + this._version;
        }
        return 'im_wheel_red.png?v=' + this._version;
    }
}

var teleop_screen = {
    command_turn: null,
    command_ctl: null,
    server_message_listeners: [],
    in_debug: 0,
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

    _select_camera: function(name) {
        var is_selected = name == this.active_camera;
        this.selected_camera = name;
        this.camera_selection_listeners.forEach(function(cb) {cb(is_selected);});
    },

    _cycle_camera_selection: function(direction) {
        if (this.selected_camera == this.active_camera) {
            this._select_camera(null);
        } else {
            this._select_camera(this.active_camera);
        }
        this.camera_cycle_timer = null;
    },

    _schedule_camera_cycle: function(direction) {
        if (this.camera_cycle_timer == undefined) {
            this.camera_cycle_timer = setTimeout(function() {teleop_screen._cycle_camera_selection();}, 250);
        }
    },

    _screen_refresh: function(message) {
        $('span#pilot_steering').text(message.ste.toFixed(3));
        $('span#pilot_throttle').text(message.thr.toFixed(3));
        // It may be the inference service is not yet available.
        const _debug = this.in_debug;
        if (message.inf_surprise != undefined) {
            $('span#inference_brake_critic').text(message.inf_brake_critic.toFixed(2));
            $('span#inference_obstacle').text(message.inf_brake.toFixed(2));
            $('span#inference_surprise').text(message.inf_surprise.toFixed(3));
            $('span#inference_critic').text(message.inf_critic.toFixed(3));
            $('span#inference_fps').text(message.inf_hz.toFixed(0));
        }
        //
        if (this.command_turn != message.turn) {
            this.command_turn = message.turn;
            $('img#arrow').attr('src', screen_utils._turn_arrow_url(message.turn));
        }
        // speed is the desired speed
        // vel_y is the actual vehicle speed
        var el_alpha_speed = $('div#alpha_speed_value');
        var el_alpha_speed_label = $('div#alpha_speed_label');
        var el_beta_speed_container = $('div#beta_speed');
        var el_beta_speed = $('div#beta_speed_value');
        if (message._is_on_autopilot) {
            el_alpha_speed.text(message.max_speed.toFixed(1));
            el_beta_speed.text(message.speed.toFixed(1));
        } else {
            el_alpha_speed.text(message.vel_y.toFixed(1));
        }
        //
        var el_steering_wheel = $('img#steeringWheel');
        var el_autopilot_status = $('#autopilot_status');
        var str_command_ctl = message.ctl + '_' + message._has_passage;
        if (this.command_ctl != str_command_ctl) {
            this.command_ctl = str_command_ctl;
            el_steering_wheel.attr('src', screen_utils._steering_wheel_url(message));
            if (message._is_on_autopilot) {
                el_alpha_speed_label.text('MAX');
                el_beta_speed_container.show();
                el_autopilot_status.text('AUTO');
            } else {
                el_alpha_speed_label.text('km/h');
                el_beta_speed_container.hide();
                el_autopilot_status.text('TELE');
            }
        }
        var display_rotation = Math.floor(message.ste * 90.0)
        el_steering_wheel.css('transform', "rotate(" + display_rotation + "deg)");
    },

    _server_message: function(message) {
        this._screen_refresh(message);
    },

    add_camera_activation_listener: function(cb) {
        this.camera_activation_listeners.push(cb);
    },

    add_camera_selection_listener: function(cb) {
        this.camera_selection_listeners.push(cb);
    },

    activate_camera: function(name) {
        this.active_camera = name;
        this._select_camera(null);
        this.camera_activation_listeners.forEach(function(cb) {cb(name);});
    },

    controller_update: function(command) {
        is_connection_ok = this.is_connection_ok;
        controller_status = this.controller_status;
        c_msg_connection_lost = this.c_msg_connection_lost;
        c_msg_controller_err = this.c_msg_controller_err;
        c_msg_teleop_view_only = this.c_msg_teleop_view_only;
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
            this._schedule_camera_cycle();
        } else if (command.arrow_right) {
            this._schedule_camera_cycle();
        }
        //
        if (command.button_left) {
            setTimeout(function() {viewport_container.fadeOut(100).fadeIn(100);}, 0);
        }
    }
}

var log_controller = {
    server_message_listeners: [],

    _notify_server_message_listeners: function(message) {
        this.server_message_listeners.forEach(function(cb) {
            cb(message);
        });
    },

    add_server_message_listener: function(cb) {
        this.server_message_listeners.push(cb);
    }
}
log_controller._capture = function() {
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
            log_controller._capture();
        };
        ws.onclose = function() {
            console.log("Logger socket connection was closed.");
        };
        ws.onmessage = function(evt) {
            var message = JSON.parse(evt.data);
            // console.log(message);
            setTimeout(function() {
                log_controller._notify_server_message_listeners(screen_utils._decorate_server_message(message));
            }, 0);
            setTimeout(log_controller._capture, 40);
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
        teleop_screen.in_debug = 0;
    } else {
        teleop_screen.in_debug = 1;
    }
});

document.addEventListener("DOMContentLoaded", function() {
    log_controller.add_server_message_listener(function(message) {
        teleop_screen._server_message(message);
    });
});

function screen_start_all() {
    if (log_controller.socket == undefined) {
        log_controller.start_socket();
    }
}

function screen_stop_all() {
    if (log_controller.socket != undefined) {
        log_controller.stop_socket();
    }
}
