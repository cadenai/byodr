
mpeg_ws_protocol = (document.location.protocol === "https:") ? "wss://" : "ws://";
//mpeg_ws_url = mpeg_ws_protocol + document.location.hostname + ":9502";
mpeg_ws_url = ws_protocol + document.location.hostname + ":" + document.location.port + '/ws/mpeg';;
//console.log(mpeg_ws_url);

var player = new JSMpeg.Player(mpeg_ws_url,
    {canvas:document.getElementById('videoCanvas'),
     autoplay:true,
     loop: true,
     disableGl: true,
     audio: false,
     progressive:true});
