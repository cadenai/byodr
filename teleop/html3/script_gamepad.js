
var NoneController = {
    gamepad_index: 0,
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
        return navigator.getGamepads()[this.gamepad_index];
    },

    poll: function() {
        return false;
    }
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
        return true;
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
        return true;
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
        return true;
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
        result.gamepad_index = gamepad.index;
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
