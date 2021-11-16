class RealServerSocket {
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
    _capture() {
        const _instance = this;
        const _socket = _instance.socket;
        if (_socket != undefined && _socket.readyState == 1) {
            _socket.send('{}');
        }
    }
    _start_socket() {
        const _instance = this;
        const _socket = _instance.socket;
        if (_socket == undefined) {
            socket_utils.create_socket("/ws/log", false, 250, function(ws) {
                _instance.socket = ws;
                ws.attempt_reconnect = true;
                ws.is_reconnect = function() {
                    return ws.attempt_reconnect;
                }
                ws.onopen = function() {
                    console.log("Server socket connection established.");
                    _instance._capture();
                };
                ws.onclose = function() {
                    console.log("Server socket connection closed.");
                };
                ws.onmessage = function(evt) {
                    var message = JSON.parse(evt.data);
                    // console.log(message);
                    setTimeout(function() {_instance._capture();}, 40);
                    setTimeout(function() {
                        _instance._notify_server_message_listeners(screen_utils._decorate_server_message(message));
                    }, 0);
                };
            });
        }
    }
    _stop_socket() {
        const _instance = this;
        const _socket = _instance.socket;
        if (_socket != undefined) {
            _socket.attempt_reconnect = false;
            if (_socket.readyState < 2) {
                _socket.close();
            }
            _instance.socket = null;
        }
    }
}


class FakeServerSocket extends RealServerSocket {
    constructor() {
        super();
        this._running = false;
        this._num_steps = 0;
        this._navigation_path_x = [0, 0, 0, 0, 0];
        this._navigation_path_dx = [0.005, 0.005, 0.005, 0.005, 0.005];
    }
    _create_message() {
        // Update the navigation path.
        if (this._num_steps == 100000) {
            this._num_steps = 0;
            this._navigation_path_x = [0, 0, 0, 0, 0];
        } else {
            this._num_steps++;
            const _navigation_path_dx = this._navigation_path_dx;
            const _a = this._navigation_path_x.map(function (num, idx) {return num + _navigation_path_dx[idx] * Math.PI;});
            this._navigation_path_x = _a;
        }
        // Simulate the path.
        const _navigation_path = this._navigation_path_x.map(function(e) {return Math.sin(e);});
        // Simulate the server message based on the gamepad
        const gamepad_command = gamepad_controller.is_active()? gamepad_controller.get_command(): {};
        return {
            'ctl': gamepad_command.button_y? 5: 2,
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
            'nav_path': _navigation_path,
            'nav_point': "",
            'speed': 0,
            'ste': gamepad_command.steering? gamepad_command.steering: 0,
            'thr': gamepad_command.throttle? gamepad_command.throttle: 0,
            'turn': "general.fallback",
            'vel_y': 0
        };
    }
    _capture() {
        const _instance = this;
        if (_instance._running) {
            var message = _instance._create_message();
            super._notify_server_message_listeners(screen_utils._decorate_server_message(message));
            setTimeout(function() {_instance._capture();}, 50);
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



class RealGamepadSocket {
    _send(command) {
        if (this.socket != undefined && this.socket.readyState == 1) {
            this.socket.send(JSON.stringify(command));
        }
    }
    _capture(server_response) {
        const gc_active = gamepad_controller.is_active();
        var gamepad_command = gc_active? gamepad_controller.get_command(): {};
        // The selected camera for ptz control can also be undefined.
        gamepad_command.camera_id = -1;
        if (teleop_screen.selected_camera == 'front') {
            gamepad_command.camera_id = 0;
        } else if (teleop_screen.selected_camera == 'rear') {
            gamepad_command.camera_id = 1;
        }
        this._send(gamepad_command);
        if (server_response != undefined && server_response.control == 'operator') {
            teleop_screen.controller_status = gc_active;
        } else if (server_response != undefined) {
            teleop_screen.controller_status = 2;
        }
        teleop_screen.controller_update(gamepad_command);
    }
    _start_socket() {
        const _instance = this;
        if (_instance.socket == undefined) {
            socket_utils.create_socket("/ws/ctl", false, 250, function(ws) {
                _instance.socket = ws;
                ws.attempt_reconnect = true;
                ws.is_reconnect = function() {
                    return ws.attempt_reconnect;
                }
                ws.onopen = function() {
                    //console.log("Operator socket connection was established.");
                    teleop_screen.is_connection_ok = 1;
                    _instance._capture();
                };
                ws.onclose = function() {
                    teleop_screen.is_connection_ok = 0;
                    teleop_screen.controller_update({});
                    //console.log("Operator socket connection was closed.");
                };
                ws.onerror = function() {
                    teleop_screen.controller_status = gamepad_controller.is_active();
                    teleop_screen.is_connection_ok = 0;
                    teleop_screen.controller_update({});
                };
                ws.onmessage = function(evt) {
                    var message = JSON.parse(evt.data);
                    setTimeout(function() {_instance._capture(message);}, 0);
                };
            });
        }
    }
    _stop_socket() {
        const _instance = this;
        if (_instance.socket != undefined) {
            _instance.socket.attempt_reconnect = false;
            if (_instance.socket.readyState < 2) {
                _instance.socket.close();
            }
            _instance.socket = null;
        }
    }
}


class FakeGamepadSocket extends RealGamepadSocket {
    _send(command) {
    }
    _poll() {
        const _instance = this;
        if (_instance._running) {
            super._capture(JSON.parse('{"control":"operator"}'));
            setTimeout(function() {_instance._poll();}, 100);
        }
    }
    _start_socket() {
        teleop_screen.is_connection_ok = 1;
        this._running = true;
        this._poll();
    }
    _stop_socket() {
        this._running = false;
        teleop_screen.is_connection_ok = 0;
        teleop_screen.controller_update({});
    }
}


// In development mode there is no use of a backend.
const server_socket = dev_tools.is_develop()? new FakeServerSocket(): new RealServerSocket();
const gamepad_socket = dev_tools.is_develop()? new FakeGamepadSocket(): new RealGamepadSocket();

function teleop_start_all() {
    gamepad_controller.reset();
    server_socket._start_socket();
    gamepad_socket._start_socket();
}

function teleop_stop_all() {
    server_socket._stop_socket();
    gamepad_socket._stop_socket();
    gamepad_controller.reset();
}


document.addEventListener("DOMContentLoaded", function() {
    server_socket.add_server_message_listener(function(message) {
        teleop_screen._server_message(message);
    });
});
