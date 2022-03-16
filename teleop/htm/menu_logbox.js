
var menu_logbox = {

    _init: function(el_parent) {
        // Construct the dom elements.
        const el_table = $("<table/>", {id: 'logbox', class: 'display', style: 'width: 100%'});
        const el_head = $("<thead/>");
        const el_head_row = $("<tr/>");
        el_head_row.append($("<th/>").text("time"));
        el_head_row.append($("<th/>").text("trigger"));
        el_table.DataTable({
            "processing": true,
            "serverSide": true,
            "ajax": "api/data/table/v10"
        });
    }
}



// --------------------------------------------------- Init and public --------------------------------------------------------- //

function menu_user_logbox_main(el_parent) {
    menu_logbox._init(el_parent);
}