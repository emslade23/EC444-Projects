//Author: Amy Dong, Quianna Mortimer, Elizabeth Slade
//////////////////////////
// Required module
var dgram = require('dgram');
const readline = require('readline');
var http = require('http');
var steps =  new Array();
var battery =  new Array();
var temperature =  new Array();
var command = "none";
var zero_num = 0;

var temp_checkbox = false;
var battery_checkbox = false;
var step_checkbox = false;

var PORT = 3030;
var HOST = '192.168.1.146';

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

// Bind server to port and IP
server.bind(PORT, HOST);

/////////////////////////

const express = require('express')
const app = express()

app.use(express.urlencoded())

var http = require('http').Server(app);

function toogleDataSeries(e){
	if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else{
		e.dataSeries.visible = true;
	}
	chart.render();
}
var empty = [];

var chartOptions = {
	title: {
		text: "Wearable Computing"
	},
	axisX: {
		ValueFormatString: "#####",
    interval: 1,
    title: "Time (seconds)"
	},
	axisY:
  [{
		title: "Steps",
		suffix: "steps"
	},
	{
    title: "Temperature",
    suffix: "ºC"
	}],
  axisY2: {
    title: "Voltage",
    suffix: "mV"
  },
	toolTip: {
		shared: true
	},
	legend: {
		cursor: "pointer",
		verticalAlign: "top",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
		itemclick: toogleDataSeries
	},
	data: [
	{
		type: "line",
		name: "Steps",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.00stepa",
		xValueFormatString: "##0s",
		axisYIndex: 0,
		dataPoints: steps
	},
	{
		type: "line",
		axisYType: "secondary",
		name: "Battery",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.00mV",
		xValueFormatString: "##0s",
		dataPoints: battery
	},
	{
		type: "line",
		name: "Temperature",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.0ºC",
		xValueFormatString: "##0s",
		axisYIndex: 1,
		dataPoints: temperature
	}]
};



var io = require('socket.io')(http);

app.get('/', function(req, res){
  res.sendFile(__dirname + '/sensors.html');
});

app.post('/', function(req, res){
	command = req.body.command;
	console.log(req.body.command);
	//res.render('/sensors.html',{ waterAlert: waterAlert});
	res.sendFile(__dirname + '/sensors.html');
})

setInterval(function(){
    io.emit('dataMsg', chartOptions);
}, 1000);

io.on('connection', function(socket){
  io.emit('dataMsg', chartOptions);
  //socket.on('disconnect', function() {
    //console.log('user disconnected');
  //});
  socket.on('temp check', function(msg){
  temp_checkbox = msg;
  //console.log(typeof msg);
  });
  socket.on('battery check', function(msg){
  battery_checkbox = msg;
  });
  socket.on('step check', function(msg){
  step_checkbox = msg;
  })
});

var starttime = null;

function readData(string) {
	//string = string.split(",");
	time = parseInt(string[3], 10);
	if (starttime == null) {
		starttime = time;
	}
	time -= starttime;

if(step_checkbox == false) {  
  steps.push({
  x: time,
  y: parseFloat(string[2])
  });
  } else {
  steps.push({
  x: time,
  y: zero_num
  });
}

if(battery_checkbox == false) {  
  battery.push({
  x: time,
  y: parseFloat(string[1])
  });
  } else {
  battery.push({
  x: time,
  y: zero_num
  });
}

if(temp_checkbox == false) {  
  temperature.push({
  x: time,
  y: parseFloat(string[0])
  });
  } else {
  temperature.push({
  x: time,
  y: zero_num
  });
}

//changes the time axis
  if (battery.length > 30) {
		steps.shift();
		battery.shift();
		temperature.shift();
	}

}

http.listen(8080, function(){
 console.log('graphing at port 8080');
});
