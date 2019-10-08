var rangeFinderData = new Array();
var ultrasonicData = new Array();
var temperatureData = new Array();

// READING DATA

var SerialPort = require('serialport');// include the library
// get port name from the command line:
var portName = '/dev/cu.SLAB_USBtoUART';
var myPort = new SerialPort(portName, {
  baudRate: 115200
});

//var rangeFinderData.push()

var Readline = SerialPort.parsers.Readline; // make instance of Readline parser
var parser = new Readline(); // make a new parser to read ASCII lines
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
  var parsed = data.split(",");
  var trimmed = parsed[0].trim();
  rangeFinderData.push(parsed[0]);
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
