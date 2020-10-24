var ws;
const STOP = 0;
const FORWARD = 1;
const BACKWARD = 2;
const LEFT = 3;
const RIGHT = 4;

 function log(line) {
    document.getElementById('log').innerHTML = document.getElementById('log').innerHTML + line + '\n';
    document.getElementById('log').scrollTop = document.getElementById('log').scrollHeight;
 }
 function clearLog() {
     document.getElementById('log').innerHTML = '';
 }

function handleVideoCheckboxClick(cb) {
    if(cb.checked){
        ws.send("video;on");
        log('Tx: Video on');
    }
    else{
        ws.send("video;off");
        log('Tx: Video off');
    }
}
 
 function updateControls() {
    document.getElementById('speedvalue').value = document.getElementById('speedslider').value;
 }
 
 function updateTargetControls() {
    document.getElementById('distancetarget').value = document.getElementById('target').value;
 }

 function sendMessage() {
     if(ws != null){
         ws.send(document.getElementById('input').value);
     }
     log('Tx: '+document.getElementById('input').value);
     document.getElementById('input').value = '';
 }

 function exit() {
     if(ws != null){
         ws.send("exit");
     }
     log('Tx: exit');
 }
 
 function move(direction) {
     var command = ''
     if(direction == FORWARD) command = "MOVE;"+document.getElementById('speedslider').value+";"+document.getElementById('speedslider').value+";"+document.getElementById('target').value+";"+document.getElementById('target').value+";30"
     if(direction == BACKWARD) command = "MOVE;-"+document.getElementById('speedslider').value+";-"+document.getElementById('speedslider').value+";"+document.getElementById('target').value+";"+document.getElementById('target').value+";30"
     if(direction == LEFT) command = "MOVE;-"+document.getElementById('speedslider').value/2+";"+document.getElementById('speedslider').value/2+";"+document.getElementById('target').value+";"+document.getElementById('target').value+";30"
     if(direction == RIGHT) command = "MOVE;"+document.getElementById('speedslider').value/2+";-"+document.getElementById('speedslider').value/2+";"+document.getElementById('target').value+";"+document.getElementById('target').value+";30"
     if(ws != null){
         ws.send(command);
     }
     log('Tx: '+command);
 }
 
 function stop() {
     if(ws != null){
         ws.send("STOP;");
     }
     log('Tx: STOP');
 }
 
 function movehead() {
    if(ws != null){
         ws.send("HEAD;"+document.getElementById('pan').value+";"+document.getElementById('tilt').value);
         log('Tx: Head('+document.getElementById('pan').value+", "+document.getElementById('tilt').value+")");
    }
 }

 function WebSocketTest() {

    if ("WebSocket" in window) {

       ws = new WebSocket('ws://' + location.host+"/websocket");

       ws.onopen = function() {

          document.getElementById("dist").style.backgroundColor = "green";
          log('Connection opened');
       };

       ws.onmessage = function (evt) {
          var obj;
          try {
                 obj = JSON.parse(evt.data);
                 document.getElementById('dist').value = obj.distance;
                 document.getElementById('speedleft').value = obj.speedL;
                 document.getElementById('posleft').value = obj.encoderL;
                 document.getElementById('speedright').value = obj.speedR;
                 document.getElementById('posright').value = obj.encoderR;
                 document.getElementById('yaw').value = obj.yaw;
                 document.getElementById('odo').value = obj.odoDistance;
              } catch(e) {
                 //document.getElementById('log').innerHTML += 'Rx: '+evt.data+'\n';
                 //log('Rx ok.');
                 document.getElementById("video").src = "data:image/jpeg;base64," + evt.data;
              }
       };

       ws.onerror = function(event) {
          console.error("WebSocket error observed:", event);
          log('Error: '+event.data);
       };

       ws.onclose = function() {
          document.getElementById("dist").style.backgroundColor = "red";
          log('Connection closed');
       };
    } else {
       // The browser doesn't support WebSocket
       alert("WebSocket NOT supported by your Browser!");
    }
 }
