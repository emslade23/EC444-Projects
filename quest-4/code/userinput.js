//////////////////////////
// Required module
var dgram = require('dgram');
const readline = require('readline');
var http = require('http');

var command = "none";
var zero_num = 0;

var StartStop_checkbox = false;


var PORT = 3030;
var HOST = '192.168.1.144';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    //console.log(remote.address + ':' + remote.port +' - ' + message);
    var str = message.toString();
    var string = str.split(",");
    readData(string);

// send command
    console.log(string);
      server.send(command,remote.port,remote.address,function(error){
      });

});

// // Bind server to port and IP
server.bind(PORT, HOST);

/////////////////////////

const express = require('express')
const app = express()

app.use(express.urlencoded())

var http = require('http').Server(app);

var io = require('socket.io')(http);

app.get('/', function(req, res){
  res.sendFile(__dirname + '/sensors.html');
});


io.on('connection', function(socket){

  socket.on('Start', function(msg){
	if (msg){
		console.log("Start the car!!!")
		command = "Start"
	}
	else{
		console.log("Stop the car!!")
		command = "Stop"
	}
  });

});

var starttime = null;


http.listen(8080, function(){
 console.log('graphing at port 8080');
});
