class RealControlsBackend {
    constructor() {
    }
    _call_get_channel_config(cb) {
        $.get("/teleop/pilot/controls/relay/conf", function(response) {cb(response['config']);});
    }
    _call_get_channel_states(cb) {
        $.get("/teleop/pilot/controls/relay/state", function(response) {cb(response['states']);});
    }
    _call_save_channel_state(channel, value, cb) {
        const command = {'channel': channel, 'action': ((value == true || value == 'true')? 'on': 'off')};
        $.post("/teleop/pilot/controls/relay/state", JSON.stringify(command)).done(function(response) {
            cb(response['channel'], response['value']);
        });
    }
}

class FakeControlsBackend extends RealControlsBackend {
    constructor() {
        super();
        this._config = [false, false, true, false];
        this._states = [false, false, false, true];
    }
    _call_get_channel_config(cb) {
        cb(this._config);
    }
    _call_get_channel_states(cb) {
        cb(this._states);
    }
    _call_save_channel_state(channel, value, cb) {
        this._states[channel] = value;
        cb(channel, value);
    }
}


var menu_controls = {
    _backend: null,

    _init: function(el_parent) {
        // In development mode there is no use of a backend.
        this._backend = dev_tools.is_develop()? new FakeControlsBackend(): new RealControlsBackend();
        // Construct the dom elements.
        const div_column = $("<div/>", {id: 'column'});
        div_column.append($("<h2/>").text("Primary relays"));
        const div_relays = $("<div/>", {id: 'relay_channels'});
        div_column.append(div_relays);
        el_parent.append(div_column);
        this._channels = [];
        this._channels.push(this._create_channel(div_relays, 3));
        this._channels.push(this._create_channel(div_relays, 4));
        this._apply_backend_config();
        this._apply_backend_states();
    },

    _apply_backend_config: function() {
        const channels = menu_controls._channels;
        menu_controls._backend._call_get_channel_config(function(config) {
            channels.forEach(function(channel) {
                if (config[channel._index]) {
                    channel.setPulsed();
                }
            });
        });
    },

    _apply_backend_states: function() {
        const channels = menu_controls._channels;
        channels.forEach(function(channel) {
            channel.flag();
            channel.disable();
        });
        menu_controls._backend._call_get_channel_states(function(states) {
            channels.forEach(function(channel) {
                channel.enable();
                channel.setValue(states[channel._index]);
                channel.un_flag();
            });
        });
    },

    _on_channel_select: function(channel) {
        // This method is called when the user sets a value and also when a value is set by code.
        if (!channel.isFlagged()) {
            menu_controls._backend._call_save_channel_state(channel._index + 1, channel.getValue(), function(ch, x) {});
            setTimeout(function() {menu_controls._apply_backend_states();}, 250);
        }
    },

    _create_channel: function(el_parent, number){
        const _channel_container = $("<p/>");
        const _name = 'channel' + number;
        const _field = $("<fieldset/>", {id: _name});
        const _legend = $("<legend/>", {id: _name + '_label'}).text('Channel ' + number);
        _field.append(_legend);
        _field.append($("<input/>", {id: _name + '_off', name: _name, type: 'radio', value: false, checked: true}));
        _field.append($("<label/>", {for: _name + '_off'}).text('Off'));
        _field.append($("<input/>", {id: _name + '_on', name: _name, type: 'radio', value: true}));
        _field.append($("<label/>", {for: _name + '_on'}).text('On'));
        _channel_container.css('width', '200px');
        _channel_container.css('line-height', '25px');
        _channel_container.append(_field)
        el_parent.append(_channel_container);

        const _slider = _field.radioslider({
            fit: true,
            onSelect: function(event) {menu_controls._on_channel_select(_slider);}
        });
        _slider._index = number - 1;
        _slider._legend = _legend;
        _slider._parent = _field;
        _slider._flagged = false;
        _slider.flag = function() {
            _slider._flagged = true;
            _slider._parent.css('cursor', 'wait');
            return _slider;
        }
        _slider.un_flag = function() {
            _slider._flagged = false;
            _slider._parent.css('cursor', 'auto');
            return _slider;
        }
        _slider.isFlagged = function() {
            return _slider._flagged;
        }
        _slider.setPulsed = function() {
            _slider._legend.text(_slider._legend.text() + " (PULSED)");
            return _slider;
        }
        _slider.isDisabled = function() {
            return _slider.find("input[name=" + _name + "]").is(':disabled');
        }
        _slider.getValue = function() {
            return _slider.find("input:checked").val();
        }
        _slider.on = function() {
            _slider.radioslider('setValue', 'true');
            return _slider;
        }
        _slider.off = function() {
            _slider.radioslider('setValue', 'false');
            return _slider;
        }
        _slider.setValue = function(x) {
            if (x || x  == 'true') {
                _slider.on();
            } else {
                _slider.off();
            }
            return _slider;
        }
        _slider.disable = function() {
            _slider.radioslider('setDisabled');
            return _slider;
        }
        _slider.enable = function() {
            _slider.radioslider('setEnabled');
            return _slider;
        }
        return _slider;
    }
}



// --------------------------------------------------- Init and public --------------------------------------------------------- //

function menu_user_controls_main(el_parent) {
    menu_controls._init(el_parent);
}