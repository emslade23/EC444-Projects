const express = require('express');
const app = express();
var dgram = require('dgram');
var command = "no command";
var split_time = [0, 1];

app.get('/', function(req, res){
    res.sendFile(__dirname + '/table.html');
  });
  
//******************************************************************* */
var http = require('http').Server(app);
http.listen(8080, function(){
    console.log('graphing at port 8080');
   });

var PORT = 3030;
var HOST = '192.168.1.145';


// Create socket
var server = dgram.createSocket('udp4');
   
// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
            server.send(command,remote.port,remote.address,function(error){
            });
});

// // Bind server to port and IP
server.bind(PORT, HOST);
//******************************************************************* */
var io = require('socket.io')(http);
io.on('connection', function(socket){
    io.emit('split_time', split_time);
});
setInterval(function(){
        io.emit('split_time', split_time);
    }, 1000);

io.on('connection', function(socket){
    socket.on('left', function(msg){
        command = msg;
        console.log('left?: ' + msg);
    });
    socket.on('right', function(msg){
        command = msg;
        console.log('right?: ' + msg);
      });
    socket.on('forward', function(msg){
        command = msg;
        console.log('forward?: ' + msg);
    });
    socket.on('backward', function(msg){
        command = msg;
        console.log('backward?: ' + msg);
      });
    socket.on('stop', function(msg){
        command = msg;
        console.log('stop?: ' + msg);
    });
    socket.on('mode', function(msg){
        command = msg;
        console.log('mode? ' + msg);
        });
  });
