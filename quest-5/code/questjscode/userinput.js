//////////////////////////
// Required module
var dgram = require('dgram');
const readline = require('readline');
var http = require('http');
//var fobID =  new Array();
var line;
var command = "yes";


const express = require('express')
const app = express()

app.use(express.urlencoded())

var http = require('http').Server(app);

app.get('/', function(req, res){
    res.sendFile(__dirname + '/table.html');
  });

  app.post('/', function(req, res){
      command = req.body.command;
      console.log(req.body.command);
      res.sendFile(__dirname + '/table.html');
  })
//******************************************************************* */
var http = require('http').Server(app);
http.listen(8080, function(){
    console.log('graphing at port 8080');
   });

//var StartStop_checkbox = false;

var io = require('socket.io')(http);
io.on('connection', function(socket){
    io.emit('fob1', fob1);
    io.emit('fob2', fob2);
    io.emit('fob3', fob3);
});
setInterval(function(){
        io.emit('fob1', fob1);
        io.emit('fob2', fob2);
        io.emit('fob3', fob3);
    }, 100);

    var db = require('./db');
    // var lineReader = require('readline').createInterface({
    //     input: require('fs').createReadStream('smoke.txt')
    //   });

    var sensorID1 = {};
    var sensorID2 = {};
    var sensorID3 = {};
    var sensorID;

    var sensor1 = [];
    var sensor2 = [];
    var sensor3 = [];

    var time1 = [];
    var time2 = [];
    var time3 = [];

    var name1 = [];
    var name2 = [];
    var name3 = [];

    var hub1 = [];
    var hub2 = [];
    var hub3 = [];

    var location1 = [];
    var location2 = [];
    var location3 = [];

    var fob1 = {};
    var fob2 = {};
    var fob3 = {};


var PORT = 3030;
var HOST = '192.168.1.131';


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
    if(remote.address == "192.168.1.144") {
      var str = message.toString();
      line = str.split(",");
      readSmokeData(line);
      DatabaseUsage();
    } else if (remote.address != "192.168.1.144") {
      if (parseInt[1] = 80085) {
          console.log("Access granted");
          server.send(command,remote.port,remote.address,function(error){
          });
      }
    }

});

// // Bind server to port and IP
server.bind(PORT, HOST);

/////////////////////////



function readSmokeData(line){
              // read smoke data and format the data
          //lineReader.on('line', function (line) {
          //console.log(line);
              sensorID = parseInt(line[0]);
              var ti = new Date();
              time = ti.toLocaleString();
              if (parseInt(line[1]) == 1) {
                var hublocation = "Front Door";
              } else if (parseInt(line[1]) == 2) {
                var hublocation = "Garage Door";
              }

              //console.log(line[5])
                  switch(sensorID) {
                  case 1:

                    time1.push(time);
                    sensor1.push(1);
                    hub1.push(parseInt(line[1]));
                    name1.push("Amy");
                    location1.push(hublocation);

                      sensorID1 = {
                          time: time1,
                          sensor: sensor1,
                          hub: hub1,
                          name: name1,
                          location: location1

                      };
                  break;
                  case 2:
                    time2.push(time);
                    sensor2.push(2);
                    hub2.push(parseInt(line[1]));
                    name2.push("Quianna");
                    location2.push(hublocation);


                    sensorID2 = {
                        time: time2,
                        sensor: sensor2,
                        hub: hub2,
                        name: name2,
                        location: location2
                      };
                  break;
                  case 3:
                    time3.push(time);
                    sensor3.push(3);
                    hub3.push(parseInt(line[1]));
                    name3.push("Lizzy");
                    location3.push(hublocation);


                    sensorID3 = {
                        time: time3,
                        sensor: sensor3,
                        hub: hub3,
                        name: name3,
                        location: location3
                      };
                  break;
                  default:
              }
          //});
  }

async function DatabaseUsage(){
      // store formatted data into the database
          let promise = new Promise((resolve, reject) => {
              setTimeout(() => resolve("done!"), 500)
            });

          let result = await promise; // wait until the promise resolves (*)
         // console.log(result, "after db:", sensorID1);

          db.put(1, sensorID1);
          db.put(2, sensorID2);
          db.put(3, sensorID3);

          db.get(1, function(err, value) {
                  if (err) {
                  console.error("null");
                  }
                  else{
                      console.log("Sensor 1:", value);
                      fob1 = value;
                  }
            });
          db.get(2, function(err, value) {
                  if (err) {
                  console.error("null");
                  }
                  else{
                      console.log("Sensor 2:", value);
                      fob2 = value;
                  }
              });
          db.get(3, function(err, value) {
                  if (err) {
                  console.error("null");
                  }
                  else{
                      console.log("Sensor 3:", value);
                      fob3 = value;
                  }
              });
      }
