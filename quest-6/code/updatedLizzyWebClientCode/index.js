const express = require('express');
const app = express();
app.use(express.static('public'))
var dgram = require('dgram');
var command = "no command";

app.get('/', function(req, res){
    res.sendFile(__dirname + '/table.html');
  });
  
//******************************************************************* */
var http = require('http').Server(app);
http.listen(8080, function(){
    console.log('graphing at port 8080');
   });
var PORT = 3030;
var HOST = '192.168.1.111';
var dgram = require('dgram');
sendMessage(command);

var splitData = 
{
    time: [],
    color: [],
    id: []
}
var inputSplit = [];
//time,color,id
// var input = "34,red,3";
// var inputSplit = input.split(',');

// splitData.time.push(inputSplit[0]);
// splitData.color.push(inputSplit[1]);
// splitData.id.push(inputSplit[2]);
// console.log(splitData)

function sendMessage(command){
    var message = new Buffer(command);

    var client = dgram.createSocket('udp4');
    client.on('message',function(msg,info){
        console.log('Data received from server : ' + msg.toString());
        console.log('Received %d bytes from %s:%d\n',msg.length, info.address, info.port);
        inputSplit = msg.toString().split(',');
        splitData.time.push(inputSplit[0]);
        splitData.color.push(inputSplit[1]);
        splitData.id.push(inputSplit[2]);
        console.log(splitData)
      });

    client.send(message, 0, message.length, PORT, HOST, function(err, bytes) {
        if (err) throw err;
        console.log('UDP message '+message+ ' sent to ' + HOST +':'+ PORT);
        });
}
//******************************************************************* */
var io = require('socket.io')(http);
io.on('connection', function(socket){
    io.emit('splitData', splitData);
});
setInterval(function(){
        io.emit('splitData', splitData);
    }, 1000);

io.on('connection', function(socket){
    socket.on('left', function(msg){
        command = msg;
        sendMessage(command);
        console.log('left?: ' + msg);
    });
    socket.on('right', function(msg){
        command = msg;
        sendMessage(command);
        console.log('right?: ' + msg);
      });
      socket.on('speedup', function(msg){
        command = msg;
        sendMessage(command);
        console.log('speedup?: ' + msg);
      });
    socket.on('straight', function(msg){
        command = msg;
        sendMessage(command);
        console.log('straight?: ' + msg);
    });
    socket.on('backward', function(msg){
        command = msg;
        sendMessage(command);
        console.log('backward?: ' + msg);
      });
    socket.on('stop', function(msg){
        command = msg;
        sendMessage(command);
        console.log('stop?: ' + msg);
    });
    socket.on('mode', function(msg){
        command = msg;
        sendMessage(command);
        console.log('mode? ' + msg);
        });
  });
