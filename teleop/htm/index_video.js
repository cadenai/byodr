

            <video id="player" autoplay muted style="width: 100% !important;"></video>


var jmuxer;
window.onload = function() {
    const port = 9001;
    const ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
    const socketURL = ws_protocol + document.location.hostname + ':' + port;
    jmuxer = new JMuxer({
        node: 'player',
        mode: 'video',
        flushingTime: 0,
        fps: 25,
        debug: false,
        onError: function(data) {
            if (/Safari/.test(navigator.userAgent) && /Apple Computer/.test(navigator.vendor)) {
                jmuxer.reset();
            }
        }
     });

     var ws = new WebSocket(socketURL);
     ws.binaryType = 'arraybuffer';
     ws.addEventListener('message',function(event) {
          jmuxer.feed({
            video: new Uint8Array(event.data)
          });
     });

     ws.addEventListener('error', function(e) {
        console.log('Socket Error');
     });
 }
