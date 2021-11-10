
class BaseServerController {
    constructor() {
        this.server_message_listeners = [];
    }
    _notify_server_message_listeners(message) {
        this.server_message_listeners.forEach(function(cb) {
            cb(message);
        });
    }
    add_server_message_listener(cb) {
        this.server_message_listeners.push(cb);
    }
}

class FakeServerController extends BaseServerController {
    constructor() {
        super();
        this._running = false;
    }
    _create_message() {
        return message = {
            'ctl': 2,
            'geo_head': -85.13683911988542,
            'geo_lat': 49.00155759398021,
            'geo_long': 8.002592177745152,
            'head': -85.13683911988542,
            'inf_brake': 0.031192919239401817,
            'inf_brake_critic': 0.7258226275444031,
            'inf_critic': 0.38173234462738037,
            'inf_hz': 29.67614871413384,
            'inf_penalty': 0.031138078539692818,
            'inf_surprise': 0.24887815117835999,
            'max_speed': 0,
            'nav_active': 0,
            'nav_command': 0,
            'nav_direction': -0.05672478172928095,
            'nav_distance': [1, 1],
            'nav_image': [-1, -1],
            'nav_path': [-0.06700272485613823, -0.0604995172470808, -0.055837567895650864, -0.05203636270016432, -0.04824773594737053],
            'nav_point': "",
            'speed': 0,
            'ste': 0,
            'thr': 0,
            'turn': "general.fallback",
            'vel_y': 0
        };
    }
    _capture() {
        if (this._running) {
            var message = this._create_message();
            this._notify_server_message_listeners(screen_utils._decorate_server_message(message));
            setTimeout(server_controller._capture, 100);
        }
    }
    _start_socket() {
        this._running = true;
        this._capture();
    }
    _stop_socket() {
        this._running = false;
    }
}

class RealServerController extends BaseServerController {
    _capture() {
        const _socket = server_controller.socket;
        if (_socket != undefined && _socket.readyState == 1) {
            _socket.send('{}');
        }
    }
    _start_socket() {
        const _socket = server_controller.socket;
        if (_socket == undefined) {
            socket_utils.create_socket("/ws/log", false, 100, function(ws) {
                server_controller.socket = ws;
                ws.attempt_reconnect = true;
                ws.is_reconnect = function() {
                    return ws.attempt_reconnect;
                }
                ws.onopen = function() {
                    console.log("Server socket connection established.");
                    server_controller._capture();
                };
                ws.onclose = function() {
                    console.log("Server socket connection closed.");
                };
                ws.onmessage = function(evt) {
                    var message = JSON.parse(evt.data);
                    // console.log(message);
                    setTimeout(server_controller._capture, 40);
                    setTimeout(function() {
                        server_controller._notify_server_message_listeners(screen_utils._decorate_server_message(message));
                    }, 0);
                };
            });
        }
    }
    _stop_socket() {
        const _socket = server_controller.socket;
        if (_socket != undefined) {
            _socket.attempt_reconnect = false;
            if (_socket.readyState < 2) {
                _socket.close();
            }
            server_controller.socket = null;
        }
    }
}

// In development mode there is no use of a backend.
const server_controller = page_utils.is_develop()? new FakeServerController(): new RealServerController();


gamepad_controller._capture = function(cs_response) {
    gc = gamepad_controller;
    gc_active = gc.is_active();
    command = gc_active? gc.get_command(): {};
    // The selected camera for ptz control can also be undefined.
    command.camera_id = -1;
    if (teleop_screen.selected_camera == 'front') {
        command.camera_id = 0;
    } else if (teleop_screen.selected_camera == 'rear') {
        command.camera_id = 1;
    }
    if (gc.socket != undefined && gc.socket.readyState == 1) {
        gc.socket.send(JSON.stringify(command));
    }
    if (cs_response != undefined && cs_response.control == 'operator') {
        teleop_screen.controller_status = gc_active;
    } else if (cs_response != undefined) {
        teleop_screen.controller_status = 2;
    }
    teleop_screen.controller_update(command);
}
gamepad_controller._start_socket = function() {
    if (gamepad_controller.socket == undefined) {
        socket_utils.create_socket("/ws/ctl", false, 100, function(ws) {
            gamepad_controller.socket = ws;
            ws.attempt_reconnect = true;
            ws.is_reconnect = function() {
                return ws.attempt_reconnect;
            }
            ws.onopen = function() {
                //console.log("Operator socket connection was established.");
                teleop_screen.is_connection_ok = 1;
                gamepad_controller._capture();
            };
            ws.onclose = function() {
                teleop_screen.is_connection_ok = 0;
                teleop_screen.controller_update({});
                //console.log("Operator socket connection was closed.");
            };
            ws.onerror = function() {
                teleop_screen.controller_status = gamepad_controller.controller.poll();
                teleop_screen.is_connection_ok = 0;
                teleop_screen.controller_update({});
            };
            ws.onmessage = function(evt) {
                var message = JSON.parse(evt.data);
                setTimeout(function() {gamepad_controller._capture(message);}, 0);
            };
        });
    }
}
gamepad_controller._stop_socket = function() {
    if (gamepad_controller.socket != undefined) {
        gamepad_controller.socket.attempt_reconnect = false;
        if (gamepad_controller.socket.readyState < 2) {
            gamepad_controller.socket.close();
        }
        gamepad_controller.socket = null;
    }
}


function teleop_start_all() {
    server_controller._start_socket();
    gamepad_controller.reset();
    gamepad_controller._start_socket();
}

function teleop_stop_all() {
    server_controller._stop_socket();
    gamepad_controller.reset();
    gamepad_controller._stop_socket();
}


document.addEventListener("DOMContentLoaded", function() {
    teleop_screen.toggle_debug_values(true);
    server_controller.add_server_message_listener(function(message) {
        teleop_screen._server_message(message);
    });
});
