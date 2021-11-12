var screen_utils = {
    _version: '0.50.0',
    _arrow_images: {},
    _wheel_images: {},

    _create_image: function(url) {
        im = new Image();
        im.src = url;
        return im;
    }, 

    _init: function() {
        this._arrow_images.left = this._create_image('im_arrow_left.png?v=' + this._version);
        this._arrow_images.right = this._create_image('im_arrow_right.png?v=' + this._version);
        this._arrow_images.ahead = this._create_image('im_arrow_up.png?v=' + this._version);
        this._arrow_images.none = this._create_image('im_arrow_none.png?v=' + this._version);
        this._wheel_images.black = this._create_image('im_wheel_black.png?v=' + this._version);
        this._wheel_images.blue = this._create_image('im_wheel_blue.png?v=' + this._version);
        this._wheel_images.red = this._create_image('im_wheel_red.png?v=' + this._version);
    }, 

    _decorate_server_message: function(message) {
        message._is_on_autopilot = message.ctl == 5;
        message._has_passage = message.inf_penalty < 1;
        return message;
    },

    _turn_arrow_img: function(turn) {
        switch(turn) {
            case "intersection.left":
                return this._arrow_images.left;
            case "intersection.right":
                return this._arrow_images.right;
            case "intersection.ahead":
                return this._arrow_images.ahead;
            default:
                return this._arrow_images.none;
        }
    },

    _steering_wheel_img: function(message) {
        if (message._is_on_autopilot && message._has_passage) {
            return this._wheel_images.blue;
        }
        if (message._has_passage) {
            return this._wheel_images.black;
        }
        return this._wheel_images.red;
    }
}


var path_renderer = {

    _init: function() {
        const _instance = this;
    },

    _render_trapezoid: function(ctx, positions, fill) {
        ctx.lineWidth = 0.5;
        ctx.strokeStyle = 'rgb(255, 255, 255)';
        ctx.fillStyle = 'rgba(100, 217, 255, 0.3)';
        ctx.beginPath();
        ctx.moveTo(positions[0][0], positions[0][1]);
        ctx.lineTo(positions[1][0], positions[1][1]);
        ctx.lineTo(positions[2][0], positions[2][1]);
        ctx.lineTo(positions[3][0], positions[3][1]);
        ctx.closePath();
        ctx.stroke();
        ctx.fill();
    },

    _get_constants: function() {
        switch(dev_tools._vehicle) {
            case "rover1":
                return [400/640, 120/480, 6/640, 8/480, 0.65, 0.65, 0.8, 0.7, 65/640, 2/480];
            default:
                return [400/640, 74/480, 6/640, 8/480, 0.65, 0.65, 0.8, 0.7, 120/640, 2/480];
        }
    },

    _render_path: function(ctx, path) {
        const canvas = ctx.canvas;
        const _constants = this._get_constants();
        const tz_width = _constants[0] * canvas.width;
        const tz_height = _constants[1] * canvas.height;
        const gap = _constants[2] * canvas.width;
        const cut = _constants[3] * canvas.height;
        const taper = _constants[4];
        const height_shrink = _constants[5];
        const gap_shrink = _constants[6];
        const cut_shrink = _constants[7];
        const w_steering = _constants[8] * canvas.width;
        const h_steering = _constants[9] * canvas.height;

        // Start from the middle of the base of the trapezoid.
        var a_x = (canvas.width / 2) - (tz_width / 2);
        var a_y = canvas.height - gap;
        var b_x = a_x + tz_width;
        var b_y = a_y;
        var idx = 0;

        path.forEach(function(element) {
            // Start in the lower left corner and draw counter clockwise.
            var w_base = b_x - a_x;
            var w_off = (w_base - (w_base * taper)) / 2;
            var v_height = tz_height * (height_shrink ** idx);
            steer_dx = w_steering * element;
            steer_dy = h_steering * element;
            var c_x = b_x - w_off + steer_dx;
            var c_y = b_y - v_height + (element > 0? steer_dy: 0);
            var d_x = a_x + w_off + steer_dx;
            var d_y = a_y - v_height - (element < 0? steer_dy: 0);
            path_renderer._render_trapezoid(ctx, [[a_x, a_y], [b_x, b_y], [c_x, c_y], [d_x, d_y]]);
            // The next step starts from the top of the previous with gap.
            var c_shrink = .5 * cut * (cut_shrink ** idx);
            var g_shrink = gap * (gap_shrink ** idx);
            a_x = d_x + c_shrink;
            a_y = d_y - g_shrink;
            b_x = c_x - c_shrink;
            b_y = c_y - g_shrink;
            idx++;
        });
    }
}

var teleop_screen = {
    el_drive_bar: null,
    el_drive_values: null,
    el_pilot_bar: null,
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
    _debug_values_listeners: [],
    camera_activation_listeners: [],
    selected_camera: null,  // Select a camera for ptz control.
    camera_selection_listeners: [],
    camera_cycle_timer: null,
    last_server_message: null,

    _init() {
        this.el_drive_bar = $("div#debug_drive_bar");
        this.el_drive_values = $("div#debug_drive_values");
        this.el_pilot_bar = $("div#pilot_drive_values");
    },

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
        if (this.selected_camera != undefined) {
            console.log("Camera " + this.selected_camera + " is selected for ptz control.")
        }
        this.camera_cycle_timer = null;
    },

    _schedule_camera_cycle: function(direction) {
        if (this.camera_cycle_timer == undefined) {
            this.camera_cycle_timer = setTimeout(function() {teleop_screen._cycle_camera_selection();}, 250);
        }
    },

    _server_message: function(message) {
        this.last_server_message = message;

        $('span#pilot_steering').text(message.ste.toFixed(3));
        $('span#pilot_throttle').text(message.thr.toFixed(3));
        // It may be the inference service is not (yet) available.
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
            $('img#arrow').attr('src', screen_utils._turn_arrow_img(message.turn).src);
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
            el_steering_wheel.attr('src', screen_utils._steering_wheel_img(message).src);
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

    add_toggle_debug_values_listener: function(cb) {
        this._debug_values_listeners.push(cb);
    },

    toggle_debug_values: function(show) {
        if (show == undefined) {
            show = !this.in_debug;
        }
        if (show) {
            this.el_drive_bar.visible();
            this.el_pilot_bar.css({'cursor': 'zoom-out'});
        } else {
            this.el_drive_bar.invisible();
            this.el_pilot_bar.css({'cursor': 'zoom-in'});
        }
        this.in_debug = show? 1: 0;
        this._debug_values_listeners.forEach(function(cb) {cb(show);});
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
            message_box.addClass('error_message');
            show_message = true;
        } else if (controller_status == 0) {
            message_box.text(c_msg_controller_err);
            message_box.removeClass();
            message_box.addClass('warning_message');
            show_message = true;
        } else if (controller_status == 2) {
            message_box.text(c_msg_teleop_view_only);
            message_box.removeClass();
            message_box.addClass('warning_message');
            show_message = true;
        }
        if (show_message) {
            viewport_container.height('calc(83vh - 0px)');
            message_box.show();
        } else {
            viewport_container.height('calc(91vh - 0px)');
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
    },

    on_canvas_init: function(width, height) {
        $('span#debug_screen_dimension').text(width + 'x' + height);
    },

    canvas_update: function(ctx) {
        const message = this.last_server_message;
        if (message != undefined && this.active_camera == 'front' && message._is_on_autopilot) {
            path_renderer._render_path(ctx, message.nav_path);
        }
    }
}


// --------------------------------------------------- Initialisations follow --------------------------------------------------------- //
screen_utils._init();

teleop_screen.add_camera_selection_listener(function(is_selected) {
    $("div#viewport_container").removeClass('selected');
    if (is_selected) {
        $("div#viewport_container").addClass('selected');
    }
});

document.addEventListener("DOMContentLoaded", function() {
    teleop_screen._init();
    teleop_screen.toggle_debug_values(true);
    teleop_screen.el_pilot_bar.click(function() {teleop_screen.toggle_debug_values()});
});
