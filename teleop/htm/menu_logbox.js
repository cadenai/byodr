
var menu_logbox = {

    _data_map: {},

    //_dt_formatter: new Intl.DateTimeFormat([], {dateStyle: 'medium', timeStyle: 'full'}),
    _dt_formatter: new Intl.DateTimeFormat([], {
                     weekday: 'short',
                     year: 'numeric',
                     month: 'numeric',
                     day: 'numeric',
                     hour: 'numeric',
                     minute: 'numeric',
                     second: 'numeric',
                     fractionalSecondDigits: 3,
                     hour12: false
                     //timeZone: Intl.DateTimeFormat().resolvedOptions().timeZone
    }),

    _init: function(el_parent) {
        const column_names = [
            'time', 'image', 'trigger', 'driver', 'max speed', 'desired speed', 'velocity', 'steer', 'throttle',
            'steer intervention', 'speed intervention', 'save', 'latitude', 'longitude',
            'ap steer', 'ap brake', 'ap steer uncertainty', 'ap brake uncertainty', 'ap running uncertainty'
        ];
        // Construct the dom elements.
        const el_table = $("<table/>", {id: 'logbox', class: 'display', style: 'font-size: 0.8em'});
        const el_head = $("<thead/>");
        const el_foot = $("<tfoot/>");
        const el_head_row = $("<tr/>");
        const el_foot_row = $("<tr/>");
        el_head.append(el_head_row);
        el_foot.append(el_foot_row);
        column_names.forEach(function(column) {el_head_row.append($("<th/>").text(column));});
        column_names.forEach(function(column) {el_foot_row.append($("<th/>").text(column));});
        el_table.append(el_head);
        el_table.append(el_foot);
        el_parent.append(el_table);
        this._setup();
    },

    _string_str: function(x) {
        return x == 'null'? '': x;
    },

    _bool_str: function(x) {
        return x == 1? 'yes': '';
    },

    _float_str: function(x, precision) {
        const result = parseFloat(x).toFixed(precision);
        return isNaN(result)? '': result;
    },

    _date_time_str: function(x) {
        // return this._dt_formatter.format(new Date(x * 1e-3));
        return this._dt_formatter.formatToParts(new Date(x * 1e-3)).map(({type, value}) => {return value;}).join('');
    },

    _driver_str: function(x) {
        return x == 1? 'teleop': x == 2? 'ap': '';
    },

    _on_data_loaded: function(data) {
        var _map = {};
        data.forEach(function(row) {
            const object_id = row[0];
            const img_exists = row[2];
            const user_steer = row[8];
            const ap_steer = row[15];
            _map[object_id] = [img_exists, user_steer, ap_steer];
        });
        this._data_map = _map;
        return data;
    },

    _img_tag: function(object_id) {
        const row = this._data_map[object_id];
        const exists = row == undefined? 0: row[0];
        if (exists) {
            var _cnt = "<canvas id='" + object_id + "' width=200 height=80></canvas>";
            _cnt += "<img ";
            _cnt += "id='" + object_id + "' ";
            _cnt += "value1='" + 0 + "' ";
            _cnt += "value2='" + 0 + "' ";
            _cnt += "style='display:none' onload='menu_logbox._img_loaded(this)' ";
            _cnt += "src='api/datalog/event/v10/image?object_id=" + object_id + "'/>";
            return _cnt;
        } else {
            return "<img src='im_no_image_available.png?v=0.61.0'/>";
        }
    },

    _draw_line: function(ctx, color, x, y) {
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, y);
        ctx.stroke();
    },

    _img_loaded: function(el_image) {
        const object_id = el_image.id;
        const row = this._data_map[object_id];
        const value1 = parseFloat(row[1]);
        const value2 = parseFloat(row[2]);
        const ctx = $("canvas#" + object_id)[0].getContext('2d');
        ctx.drawImage(el_image, 0, 0, 200, 80);
        this._draw_line(ctx, '#fff', (1 + value1) * 100, 80);
        this._draw_line(ctx, '#0937b5', (1 + value2) * 100, 80);
    },

    _setup: function() {
        $('table#logbox').DataTable({
            "processing": true,
            "serverSide": true,
            "searching": false,
            ajax: {
                url: "api/datalog/event/v10/table",
                dataSrc: function (x) {return menu_logbox._on_data_loaded(x.data);}
            },
            "columns": [
                {data: 1, orderable: true, render: function(data, type, row, meta) {return menu_logbox._date_time_str(data);}},
                {data: 0, orderable: false, render: function(data, type, row, meta) {return menu_logbox._img_tag(data);}},
                {data: 3, orderable: false},
                {data: 4, orderable: false, render: function(data, type, row, meta) {return menu_logbox._driver_str(data);}},
                {data: 5, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 6, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 7, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 8, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 9, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 10, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 11, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 12, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 13, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 8);}},
                {data: 14, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 8);}},
                {data: 15, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 16, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 17, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 18, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}},
                {data: 19, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 4);}}
            ]
        });
    }
}



// --------------------------------------------------- Init and public --------------------------------------------------------- //

function menu_user_logbox_main(el_parent) {
    menu_logbox._init(el_parent);
}