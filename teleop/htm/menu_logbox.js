
var menu_logbox = {
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
        // Construct the dom elements.
        const el_table = $("<table/>", {id: 'logbox', class: 'display', style: 'font-size: 0.8em'});
        const el_head = $("<thead/>");
        const el_head_row = $("<tr/>");
        el_head_row.append($("<th/>").text("time"));
        el_head_row.append($("<th/>").text("trigger"));
        el_head_row.append($("<th/>").text("driver"));
        el_head_row.append($("<th/>").text("max speed"));
        el_head_row.append($("<th/>").text("desired speed"));
        el_head_row.append($("<th/>").text("velocity"));
        el_head_row.append($("<th/>").text("steer"));
        el_head_row.append($("<th/>").text("throttle"));
        el_head_row.append($("<th/>").text("steer intervention"));
        el_head_row.append($("<th/>").text("speed intervention"));
        el_head_row.append($("<th/>").text("save"));
        el_head_row.append($("<th/>").text("latitude"));
        el_head_row.append($("<th/>").text("longitude"));
        el_head_row.append($("<th/>").text("ap steer"));
        el_head_row.append($("<th/>").text("ap brake"));
        el_head_row.append($("<th/>").text("ap steer uncertainty"));
        el_head_row.append($("<th/>").text("ap brake uncertainty"));
        el_head_row.append($("<th/>").text("ap running uncertainty"));
        el_head.append(el_head_row);
        el_table.append(el_head);
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
        //return this._dt_formatter.format(new Date(x * 1e-3));
        return this._dt_formatter.formatToParts(new Date(x * 1e-3)).map(({type, value}) => {return value;}).join('');
    },

    _setup: function() {
        $('table#logbox').DataTable({
            "processing": true,
            "serverSide": true,
            "searching": false,
            "ajax": "api/data/table/v10",
            "columns": [
                {data: 0, orderable: true, render: function(data, type, row, meta) {return menu_logbox._date_time_str(data);}},
                {data: 1, orderable: false},
                {data: 2, orderable: false, render: function(data, type, row, meta) {return menu_logbox._string_str(data);}},
                {data: 3, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 4, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 5, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 6, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 7, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 2);}},
                {data: 8, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 9, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 10, orderable: false, render: function(data, type, row, meta) {return menu_logbox._bool_str(data);}},
                {data: 11, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 8);}},
                {data: 12, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 8);}},
                {data: 13, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 5);}},
                {data: 14, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 5);}},
                {data: 15, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 5);}},
                {data: 16, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 5);}},
                {data: 17, orderable: false, render: function(data, type, row, meta) {return menu_logbox._float_str(data, 5);}}
            ]
        });
    }
}



// --------------------------------------------------- Init and public --------------------------------------------------------- //

function menu_user_logbox_main(el_parent) {
    menu_logbox._init(el_parent);
}