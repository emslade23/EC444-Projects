const express = require('express');
const app = express();

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
//******************************************************************* */
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
    }, 1000);
//******************************************************************* */

var db = require('./db');
var lineReader = require('readline').createInterface({
    input: require('fs').createReadStream('smoke.txt')
  });

var sensorID1 = {};
var sensorID2 = {};
var sensorID3 = {};
var sensorID;

var time1 = [];
var time2 = [];
var time3 = [];

var smoke1 = [];
var smoke2 = [];
var smoke3 = [];

var latitude1 = [];
var latitude2 = [];
var latitude3 = [];

var longitude1 = [];
var longitude2 = [];
var longitude3 = [];

var fob1 = {};
var fob2 = {};
var fob3 = {};

readSmokeData();
DatabaseUsage();

async function readSmokeData(){
        // read smoke data and format the data
    lineReader.on('line', function (line) {
        sensorID = parseInt(line[5]);
        //console.log(line[5])
            switch(sensorID) {
            case 1:
                    time1.push(parseInt(line.slice(0,4)));
                    smoke1.push(parseInt(line[7]));
                    latitude1.push(parseFloat(line.slice(9, 11)));
                    longitude1.push(parseFloat(line.slice(12, line.length))); 

                sensorID1 = {
                    time: time1,
                    latitude: latitude1,
                    longitude: longitude1
                };
            break;
            case 2:
                    time2.push(parseInt(line.slice(0,4)));
                    smoke2.push(parseInt(line[7]));
                    latitude2.push(parseFloat(line.slice(9, 11)));
                    longitude2.push(parseFloat(line.slice(12, line.length))); 
    
                sensorID2 = {
                    time: time2,
                    latitude: latitude2,
                    longitude: longitude2
                };
            break;
            case 3:
                    time3.push(parseInt(line.slice(0,4)));
                    smoke3.push(parseInt(line[7]));
                    latitude3.push(parseFloat(line.slice(9, 11)));
                    longitude3.push(parseFloat(line.slice(12, line.length))); 

                sensorID3 = {
                        time: time3,
                        latitude: latitude3,
                        longitude: longitude3
                    };  
            break;
            default:
        }
    });
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


