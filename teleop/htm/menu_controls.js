class RealControlsBackend {
    constructor() {
    }
    _call_get_channel_states(cb) {
        $.get("/api/pilot/controls/relay", cb);
    }
    _call_save_channel_state(data, cb) {
        $.post("/api/pilot/controls/relay", JSON.stringify(data)).done(cb);
    }
}

class FakeControlsBackend extends RealControlsBackend {
    constructor() {
        super();
        this._states = [false, false, false, true];
    }
    _call_get_channel_states(cb) {
        cb({"states": this._states});
    }
    _call_save_channel_state(command, cb) {
        //console.log("Dev backend save channel: " + command);
        const channel = command['channel'];
        const action = command['action'];
        const value = action == 'on' ? true: false;
        this._states[channel] = value;
        cb({"channel": channel, "state": value});
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

        const p_channel4_container = $("<p/>");
        p_channel4_container.css('line-height', '25px');
        const div_channel4 = $("<div/>", {id: 'channel4'});
        div_channel4.append($("<input/>", {id: 'channel4_off', name: 'options', type: 'radio', value: false}));
        div_channel4.append($("<label/>", {for: 'channel4_off'}).text('Off'));
        div_channel4.append($("<input/>", {id: 'channel4_on', name: 'options', type: 'radio', value: true}));
        div_channel4.append($("<label/>", {for: 'channel4_on'}).text('On'));
        div_channel4.css('width', '200px');
                
        p_channel4_container.append($("<span/>", {id: 'channel4_label'}).text('Channel 4'));
        p_channel4_container.append(div_channel4)
        div_relays.append(p_channel4_container);
        div_column.append(div_relays);
        el_parent.append(div_column);

        this._channel4_slider = div_channel4.radioslider({fit: true});
        this._channel4_slider.radioslider('setValue', false);
        this._channel4_slider.radioslider('setDisabled');
        this._channel4_slider.on('radiochange', function() {
            menu_controls._channel4_slider.radioslider('setDisabled');
            const _val = $("div#channel4 input[name='options']:checked").val();
            const _action = _val == 'true'? 'on': 'off';
            const command = {'channel': 4, 'action': _action};
            menu_controls._backend._call_save_channel_state(command, function(response) {
                menu_controls._render_channel_state(menu_controls._channel4_slider, response['state']);
            });
        });

        this._backend._call_get_channel_states(menu_controls._render_relay_states);
    },

    _render_channel_state: function(element, value) {
        element.radioslider('setEnabled');
        element.radioslider('setValue', value);
    },

    _render_relay_states: function(data) {
        //console.log(data);
        menu_controls._render_channel_state(menu_controls._channel4_slider, data['states'][3]);
    }
}



// --------------------------------------------------- Init and public --------------------------------------------------------- //

function menu_user_controls_main(el_parent) {
    menu_controls._init(el_parent);
}