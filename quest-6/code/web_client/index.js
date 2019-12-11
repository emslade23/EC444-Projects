const express = require('express');
const app = express();
app.use(express.static('public'))
var dgram = require('dgram');
var command = "no command";
var db = require('./db');

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
DatabaseUsage();

var splitData = 
{
    run: [],
    time: [],
    color: [],
    id: [],
    total: []
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
        splitData.run.push(inputSplit[0]);
        splitData.time.push(inputSplit[1]);
        splitData.color.push(inputSplit[2]);
        splitData.id.push(inputSplit[3]);
        splitData.total.push(inputSplit[4]);
        DatabaseUsage();
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
    socket.on('run', function(msg){
        command = msg;
        sendMessage(command);
        console.log('run? ' + msg);
        });
  });
//pushing data to the database
async function DatabaseUsage(){
    // store formatted data into the database
        let promise = new Promise((resolve, reject) => {
            setTimeout(() => resolve("done!"), 500)
          });

        let result = await promise; // wait until the promise resolves (*)
       // console.log(result, "after db:", sensorID1);

        db.put(1, splitData);

        db.get(1, function(err, value) {
                if (err) {
                console.error("null");
                }
                else{
                    console.log("Split Time:", value);
                    splitDatabase = value;
                }
          });

    }