
gamepad_controller._capture = function(cs_response) {
    gc = gamepad_controller;
    gc_active = gc.is_active();
    command = gc_active? gc.get_command(): {};
    command.camera_id = teleop_screen.selected_camera == 'rear'? 1: 0;
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
    socket_utils.create_socket("/ws/ctl", false, 100, function(ws) {
        gamepad_controller.socket = ws;
        ws.attempt_reconnect = true;
        ws.is_reconnect = function() {
            return ws.attempt_reconnect;
        }
        ws.onopen = function() {
            console.log("Operator socket connection was established.");
            teleop_screen.is_connection_ok = 1;
            gamepad_controller._capture();
        };
        ws.onclose = function() {
            teleop_screen.is_connection_ok = 0;
            teleop_screen.controller_update({});
            console.log("Operator socket connection was closed.");
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
gamepad_controller._stop_socket = function() {
    gamepad_controller.socket.attempt_reconnect = false;
    if (gamepad_controller.socket.readyState < 2) {
        gamepad_controller.socket.close();
    }
    gamepad_controller.socket = null;
}


page_utils.add_toggle_debug_values_listener(function(collapse) {
    if (collapse) {
        $("div#debug_drive_values").invisible();
        $("div#pilot_drive_values").css({'cursor': 'zoom-in'});
    } else {
        $("div#debug_drive_values").visible();
        $("div#pilot_drive_values").css({'cursor': 'zoom-out'});
    }
});

function teleop_start_all() {
    gamepad_controller.reset();
    if (gamepad_controller.socket == undefined) {
        gamepad_controller._start_socket();
    }
}

function teleop_stop_all() {
    gamepad_controller.reset();
    if (gamepad_controller.socket != undefined) {
        gamepad_controller._stop_socket();
    }
}
