
var camera_player = {
    uri: null,
    el_canvas: null,
    wsavc: null,

    callback: {
        onopen: function(player) {
            player.playStream();
        },

        onclose: function(player) {
            setTimeout(function() {player.connect(this.uri);}, 100);
        }
    },

    init: function(parent, port) {
        this.el_canvas = document.createElement("canvas");
        parent.appendChild(this.el_canvas);
        this.uri = "ws://" + document.location.hostname + ':' + port;
        this.wsavc = new WSAvcPlayer(this.el_canvas, "webgl", this.callback);
        this.wsavc.connect(this.uri);
        window.wsavc = this.wsavc;
    }
}

document.addEventListener("DOMContentLoaded", function() {
    camera_player.init(document.getElementById('camera1'), 9101);
});
