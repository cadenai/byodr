class RealNavigatorController {
    constructor() {
        const location = document.location;
        this.nav_path = location.protocol + "//" + location.hostname + ":" + location.port + "/ws/nav";
        this.random_id = Math.random();  // For browser control of navigation image urls.
        this.el_current_image = null;
        this.el_next_image = null;
        this.el_image_width = null;
        this.el_image_height = null;
        this.el_route = null;
        this.el_point = null;
        this.el_route_select_prev = null;
        this.el_route_select_next = null;
        this.navigation_images = [null, null];
        this.routes = [];
        this.selected_route = null;
        this.started = false;
        this.backend_active = false;
        this.in_mouse_over = false;
        this.in_debug = false;
    }
    _initialize() {
        this.el_current_image = $('img#current_navigation_image');
        this.el_next_image = $('img#next_navigation_image');
        this.el_image_width = this.el_current_image.width();
        this.el_image_height = this.el_current_image.height();
        this.el_route = $('span#navigation_route_name');
        this.el_point = $('span#navigation_point_name');
        this.el_route_select_prev = $('span#navigation_route_sel_prev');
        this.el_route_select_next = $('span#navigation_route_sel_next');
        this.el_point.text('');
    }
    _has_next_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0 && (route == undefined || this.routes.indexOf(route) + 1 < t_size)) {
            return true;
        }
        return false;
    }
    _has_prev_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0 && (route == undefined || this.routes.indexOf(route) - 1 >= 0)) {
            return true;
        }
        return false;
    }
    _set_routes(response) {
        const routes = response.routes;
        const selected = response.selected;
        this.routes = routes;
        if (routes.length == 0) {
            this._select_route(null);
        } else if (routes.indexOf(selected) != -1) {
            this._select_route(selected);
        } else {
            this._select_route(routes[0]);
        }
    }
    _select_route(name) {
        this.selected_route = name;
        this.el_route.text(name);
    }
    _select_prev_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0) {
            var idx = this.routes.indexOf(route) - 1;
            this._select_route(this.routes[idx < 0 ? t_size - 1 : idx]);
        }
    }
    _select_next_route() {
        const route = this.selected_route;
        const t_size = this.routes == undefined ? 0 : this.routes.length;
        if (t_size > 0) {
            var idx = this.routes.indexOf(route) + 1;
            this._select_route(this.routes[idx >= t_size ? 0 : idx]);
        }
    }
    _render_navigation_images(current_src, next_src) {
        this.el_current_image.attr('src', current_src).width(this.el_image_width).height(this.el_image_height);
        this.el_next_image.attr('src', next_src).width(this.el_image_width).height(this.el_image_height);
    }
    _update_visibility() {
        if (this.routes.length < 1) {
            $('div#navigation_image_container').invisible();
            $('div#navigation_route_container').invisible();
        } else {
            $('div#navigation_image_container').visible();
            $('div#navigation_route_container').visible();
        }
    }
    _navigation_img_url(_id) {
        return this.nav_path + '?im=' + _id + '&r=' +  this.selected_route + '&n=' + this.random_id;
    }
    _schedule_navigation_image_update() {
        var current_src = screen_utils._navigation_icon(this.backend_active? 'pause': 'play').src
        var next_src = this._navigation_img_url(this.navigation_images[1]);
        if (this.backend_active && !this.in_mouse_over) {
            current_src = this._navigation_img_url(this.navigation_images[0]);
        }
        const _instance = this;
        setTimeout(function() {
            _instance._render_navigation_images(current_src, next_src);
        }, 0);
    }
    _mouse_over() {
        this.in_mouse_over = true;
        this._schedule_navigation_image_update();
    }
    _mouse_out() {
        this.in_mouse_over = false;
        this._schedule_navigation_image_update();
    }
    _toggle_route() {
        var command = {'action': 'toggle', 'route': this.selected_route};
        $.post('api/navigation/routes', JSON.stringify(command))
                    .fail(function(xhr, status, error) {
                        console.log(error);
                    })
                    .done(function(data) {
                        // console.log(data);
                    });
    }
    _list_routes() {
        const _instance = this;
        $.get("/api/navigation/routes?action=list", function(response) {
            _instance._set_routes(response);
            _instance._update_visibility();
        });
    }
    _server_message(message) {
        $('span#navigation_geo_lat').text(message.geo_lat.toFixed(6));
        $('span#navigation_geo_long').text(message.geo_long.toFixed(6));
        $('span#navigation_heading').text(message.geo_head.toFixed(2));
        if (this.started) {
            if (message.inf_surprise != undefined) {
                $('span#navigation_match_image_distance').text(message.nav_distance[1].toFixed(3));
                $('span#navigation_current_command').text(message.nav_command.toFixed(1));
                $('span#navigation_direction').text(message.nav_direction.toFixed(2));
            }
            const backend_active = message.nav_active;
            const is_backend_change = backend_active != this.backend_active;
            const is_image_change = message.nav_image[0] != this.navigation_images[0] || message.nav_image[1] != this.navigation_images[1];
            this.backend_active = backend_active;
            this.navigation_images = message.nav_image;
            if (is_image_change || is_backend_change) {
                this._schedule_navigation_image_update();
            }
            if (backend_active) {
                this.el_point.text(message.nav_point);
            }
        }
    }
}

class FakeNavigatorController extends RealNavigatorController {
    constructor() {
        super();
    }
    _toggle_route() {
    }
    _list_routes() {
    }
}

// In development mode there is no use of a backend.
const navigator_controller = dev_tools.is_develop()? new FakeNavigatorController(): new RealNavigatorController();

function navigator_start_all() {
    navigator_controller.started = true;
    navigator_controller._list_routes();
}

function navigator_stop_all() {
    navigator_controller.started = false;
}


// --------------------------------------------------- Initialisations follow --------------------------------------------------------- //
teleop_screen.add_toggle_debug_values_listener(function(show) {
    navigator_controller.in_debug = show;
    navigator_controller._update_visibility();
});

document.addEventListener("DOMContentLoaded", function() {
    navigator_controller._initialize();
    navigator_controller.el_current_image.mouseover(function() {navigator_controller._mouse_over()});
    navigator_controller.el_current_image.mouseout(function() {navigator_controller._mouse_out()});
    navigator_controller.el_current_image.click(function() {navigator_controller._toggle_route()});
    navigator_controller.el_route_select_prev.click(function() {navigator_controller._select_prev_route()});
    navigator_controller.el_route_select_next.click(function() {navigator_controller._select_next_route()});
    server_socket.add_server_message_listener(function(message) {
        navigator_controller._server_message(message);
    });
});
