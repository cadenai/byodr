class NavigatorController {
    constructor() {
        const location = document.location;
        this.nav_path = location.protocol + "//" + location.hostname + ":" + location.port + "/ws/nav";
        this.image_element = null;
        this.navigation_id = null;
        this.active = false;
    }
    set_element(el_image) {
        this.image_element = el_image;
    }
    update(nav_id) {
        if (this.image_element != undefined && this.active) {
            if (nav_id != this.navigation_id) {
                setTimeout(function() {
                    const img_url = navigator_controller.nav_path + '?n=' + Math.random();
                    navigator_controller.image_element.src = img_url;
                }, 10);
            }
        }
        this.navigation_id = nav_id;
    }
}
const navigator_controller = new NavigatorController();

document.addEventListener("DOMContentLoaded", function() {
    navigation_image_element = document.createElement("img");
    document.getElementById('navigation_container').appendChild(navigation_image_element);
    navigator_controller.set_element(navigation_image_element);
    log_controller.add_server_message_listener(function(message) {
        navigator_controller.update(message.nav_image);
    });
});


function navigator_start_all() {
    navigator_controller.active = true;
}

function navigator_stop_all() {
    navigator_controller.active = false;
}
