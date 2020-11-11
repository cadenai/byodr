class NavigatorController {
    constructor() {
        const location = document.location;
        this.nav_path = location.protocol + "//" + location.hostname + ":" + location.port + "/ws/nav";
        this.el_image = null;
        this.el_route = null;
        this.el_point = null;
        this.el_debug_distance = null;
        this.el_route_select_prev = null;
        this.el_route_select_next = null;
        this.matched_image_id = null;
        this.routes = [];
        this.selected_route = null;
        this.active = false;
    }
    get_selected_route() {
        return this.selected_route;
    }
    has_next_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0 && (route == undefined || this.routes.indexOf(route) + 1 < t_size)) {
            return true;
        }
        return false;
    }
    has_prev_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0 && (route == undefined || this.routes.indexOf(route) - 1 >= 0)) {
            return true;
        }
        return false;
    }
    set_routes(routes) {
        this.routes = routes;
        if (routes.length == 0) {
            this.select_route(null);
            this.el_route.text('No route available');
        } else {
            if (this.selected_route == undefined) {
                this.selected_route = routes[0];
            }
            this.select_route(this.selected_route);
        }
    }
    select_route(name) {
        this.selected_route = name;
        this.el_route.text(name);
    }
    select_prev_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0) {
            var idx = this.routes.indexOf(route) - 1;
            this.select_route(this.routes[idx < 0 ? t_size - 1 : idx]);
        }
    }
    select_next_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0) {
            var idx = this.routes.indexOf(route) + 1;
            this.select_route(this.routes[idx >= t_size ? 0 : idx]);
        }
    }
    initialize() {
        this.el_image = $('img#navigation_image');
        this.el_route = $('span#navigation_route_name');
        this.el_point = $('span#navigation_point_name');
        this.el_debug_distance = $('span#navigation_image_distance');
        this.el_route_select_prev = $('span#navigation_route_sel_prev');
        this.el_route_select_next = $('span#navigation_route_sel_next');
    }
    schedule_navigation_image_update() {
        // Randomize the url so the browser does not cache the image - it changes on the server at route switches.
        const nc = navigator_controller;
        const image_id = nc.matched_image_id;
        setTimeout(function() {nc.el_image.attr('src', nc.nav_path + '?im=' + image_id + '&n=' + Math.random());}, 10);
    }
    on_message(message) {
        if (this.active) {
            // Use the pilot properties unless in debug mode.
            const is_debug = this.el_debug_distance.is_visible();
            const column_id = is_debug ? 1 : 0;
            this.el_point.text(message.nav_point[column_id]);
            this.el_debug_distance.text(message.nav_distance.toFixed(3));
            const nav_id = message.nav_image[column_id];
            if (nav_id != this.matched_image_id) {
                this.matched_image_id = nav_id;
                this.schedule_navigation_image_update();
            }
        }
    }
}
const navigator_controller = new NavigatorController();
navigator_controller.api_route_command = function(command, fn_done) {
    $.post('api/navigation/routes', JSON.stringify(command))
                .fail(function(xhr, status, error) {console.log(error);})
                .done(fn_done)
}
navigator_controller.toggle_route = function() {
    navigator_controller.api_route_command({'action': 'toggle', 'route': this.get_selected_route()}, function(data) {})
}


document.addEventListener("DOMContentLoaded", function() {
    navigator_controller.initialize();
    navigator_controller.el_image.click(function() {navigator_controller.toggle_route()});
    navigator_controller.el_route_select_prev.click(function() {navigator_controller.select_prev_route()});
    navigator_controller.el_route_select_next.click(function() {navigator_controller.select_next_route()});
    log_controller.add_server_message_listener(function(message) {
        navigator_controller.on_message(message);
    });
});


function navigator_start_all() {
    $.get("/api/navigation/routes?action=list", function(data) {
        navigator_controller.set_routes(data);
    });
    navigator_controller.active = true;
}

function navigator_stop_all() {
    navigator_controller.active = false;
}
