class NavigatorController {
    constructor() {
        const location = document.location;
        this.nav_path = location.protocol + "//" + location.hostname + ":" + location.port + "/ws/nav";
        this.el_image = null;
        this.el_route = null;
        this.el_debug = null;
        this.matched_image_id = null;
        this.routes = null;
        this.selected_route = null;
        this.active = false;
    }
    initialize(el_image, el_route, el_debug) {
        this.el_image = el_image;
        this.el_route = el_route;
        this.el_debug = el_debug
    }
    set_routes(routes) {
        this.routes = routes;
        if (routes.length == 0) {
            this.select_route(null);
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
    get_selected_route() {
        return this.selected_route;
    }
    on_message(message) {
        if (this.el_image != undefined && this.active) {
            const nav_id = message.nav_image;
            if (nav_id != this.matched_image_id) {
                setTimeout(function() {
                    // Randomize the url so the browser does not cache the image - it may change on the server.
                    const img_url = navigator_controller.nav_path + '?n=' + Math.random();
                    navigator_controller.el_image.attr('src', img_url);
                }, 10);
            }
            this.matched_image_id = nav_id;
            this.el_debug.text(message.nav_distance.toFixed(4));
        }
    }
}
const navigator_controller = new NavigatorController();

document.addEventListener("DOMContentLoaded", function() {
    log_controller.add_server_message_listener(function(message) {
        navigator_controller.on_message(message);
    });
    navigator_controller.initialize($('img#navigation_image'), $('span#navigation_route_name'), $('span#navigation_image_distance'));
    $("span#navigation_route_name").click(function() {
        p_body = {'action': 'start', 'route': navigator_controller.get_selected_route()};
        $.post('api/navigation/routes', JSON.stringify(p_body))
        .done(function(data) {})
        .fail(function(xhr, status, error) {
            console.log(error);
        })
    });
});


function navigator_start_all() {
    jQuery.get("/api/navigation/routes?action=list", function(data) {
        navigator_controller.set_routes(data);
    });
    navigator_controller.active = true;
}

function navigator_stop_all() {
    navigator_controller.active = false;
}
