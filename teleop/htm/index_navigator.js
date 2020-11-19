class NavigatorController {
    constructor() {
        const location = document.location;
        this.nav_path = location.protocol + "//" + location.hostname + ":" + location.port + "/ws/nav";
        this.el_image = null;
        this.el_image_width = null;
        this.el_image_height = null;
        this.el_route = null;
        this.el_point = null;
        this.el_debug_match_distance = null;
        this.el_route_select_prev = null;
        this.el_route_select_next = null;
        this.matched_image_id = null;
        this.routes = [];
        this.selected_route = null;
        this.active = false;
        this.backend_active = false;
        this.in_mouse_over = false;
        this.in_debug = false;
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
            this.el_route.text('No routes');
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
    render_navigation_image(image_src) {
        this.el_image.attr('src', image_src).width(this.el_image_width).height(this.el_image_height);
    }
    initialize() {
        this.el_image = $('img#navigation_image');
        this.el_image_width = this.el_image.width();
        this.el_image_height = this.el_image.height();
        this.el_route = $('span#navigation_route_name');
        this.el_point = $('span#navigation_point_name');
        this.el_debug_match_distance = $('span#navigation_match_image_distance');
        this.el_route_select_prev = $('span#navigation_route_sel_prev');
        this.el_route_select_next = $('span#navigation_route_sel_next');
        this.schedule_navigation_image_update();
    }
    schedule_navigation_image_update() {
        var image_src = this.backend_active? 'icon_pause.png?v=0.45.0' : 'icon_play.png?v=0.45.0';
        const image_id = this.matched_image_id;
        if (this.backend_active && !this.in_mouse_over) {
            // Randomize the url so the browser does not cache the image - it changes on the server at route switches.
            image_src =  this.nav_path + '?im=' + image_id + '&n=' + Math.random();
        }
        setTimeout(function() {
            navigator_controller.render_navigation_image(image_src);
        }, 0);
    }
    on_message(message) {
        if (this.active) {
            // Show the current navigation values when in debug.
            const _debug = this.el_debug_match_distance.is_visible();
            const column_id = _debug? 1 : 0;
            const image_id = message.nav_image[column_id];
            const backend_active = message.nav_active;
            const is_image_change = image_id != this.matched_image_id;
            const is_backend_change = backend_active != this.backend_active;
            this.matched_image_id = image_id;
            this.backend_active = backend_active;
            if (is_image_change || is_backend_change) {
                this.schedule_navigation_image_update();
            }
            if (backend_active) {
                this.el_point.text(message.nav_point);
                this.el_debug_match_distance.text(message.nav_distance[column_id].toFixed(3));
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
navigator_controller.mouse_over = function() {
    navigator_controller.in_mouse_over = true;
    navigator_controller.schedule_navigation_image_update();
}
navigator_controller.mouse_out = function() {
    navigator_controller.in_mouse_over = false;
    navigator_controller.schedule_navigation_image_update();
}
navigator_controller.update_visibility = function () {
    if (navigator_controller.routes.length < 1) {
        $('div#navigation_image_container').invisible();
        $('div#navigation_route_container').invisible();
        $('div#navigation_debug_container').invisible();
    } else {
        $('div#navigation_image_container').visible();
        $('div#navigation_route_container').visible();
        if (navigator_controller.in_debug) {
            $('div#navigation_debug_container').visible();
        } else {
            $('div#navigation_debug_container').invisible();
        }
    }
}

page_utils.add_toggle_debug_values_listener(function(collapse) {
    navigator_controller.in_debug = !collapse;
    navigator_controller.update_visibility();
});

document.addEventListener("DOMContentLoaded", function() {
    navigator_controller.initialize();
    navigator_controller.el_image.mouseover(function() {navigator_controller.mouse_over()});
    navigator_controller.el_image.mouseout(function() {navigator_controller.mouse_out()});
    navigator_controller.el_image.click(function() {navigator_controller.toggle_route()});
    navigator_controller.el_route_select_prev.click(function() {navigator_controller.select_prev_route()});
    navigator_controller.el_route_select_next.click(function() {navigator_controller.select_next_route()});
    log_controller.add_server_message_listener(function(message) {
        navigator_controller.on_message(message);
    });
});

function navigator_start_all() {
    navigator_controller.active = true;
    $.get("/api/navigation/routes?action=list", function(data) {
        navigator_controller.set_routes(data);
        navigator_controller.update_visibility();
    });
}

function navigator_stop_all() {
    navigator_controller.active = false;
}
