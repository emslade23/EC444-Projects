// code to read data from text file and store it into two arrays.
const csv = require('csv-parser');
const fs = require('fs');

var rangeFinderData = new Array();
var ultrasonicData = new Array();
var temperatureData = new Array();
var parsed = new Array();
var trimmed = new Array();

// READING DATA

var SerialPort = require('serialport');// include the library
// get port name from the command line:
var portName = '/dev/cu.SLAB_USBtoUART';
var myPort = new SerialPort(portName, {
  baudRate: 115200
});

//var rangeFinderData.push()

var Readline = SerialPort.parsers.Readline; // make instance of Readline parser
var parser = new Readline({ delimiter: '\n' }); // make a new parser to read ASCII lines
myPort.pipe(parser); // pipe the serial stream to the parser

myPort.on('open', showPortOpen);
parser.on('data', readSerialData);

myPort.on('close', showPortClose);
myPort.on('error', showError);

function showPortOpen() {
  console.log('port openned!!!!!!. Data rate: ' + myPort.baudRate);
}

function readSerialData(data) {
//  console.log(data);
  parsed = data.split(",");
  trimmed = parsed[0].trim();
  rangeFinderData.push(trimmed);
  ultrasonicData.push(parsed[1]);
  temperatureData.push(parsed[2]);
  console.log(trimmed);
}

function showPortClose() {
  console.log('port closed.');
}

function showError(error) {
  console.log('Serial port error: ' + error);
}

  var express = require('express');
  var app = express();
  //app.locals.csv = csv;
  //app.locals.fs = fs;

//  console.log("haha");
// view engine setup
  app.set('view engine', 'ejs');
  app.use(express.static(__dirname + '/public'));

  app.get('/', function(req, res){
    res.render('graphs',{user: "EC444 Staff", rangeFinderData: rangeFinderData,
    ultrasonicData: ultrasonicData, temperatureData: temperatureData,
     title : "EC444 Group Number 4, Quest 2"});
  });

  app.get('/dummygraphs', function(req, res){
    res.render('dummy');
  });
  app.get('/dummygraphs2', function(req, res){
    res.render('dummy2');
  });

  app.listen(process.env.port || 3000);
