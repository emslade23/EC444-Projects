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

var temperature = [];
var battery = [];
var steps = [];
var empty = [];

var chartOptions = {
	title: {
		text: "Sensor Central"
	},
	axisX: {
		ValueFormatString: "#####",
    interval: 1,
    title: "Time (seconds)"
	},
	axisY: [ {
		title: "Steps",
		suffix: "steps"
	}],
	axisY2: 
		{
			title: "Temperature",
			suffix: "ºC"
		},
	axisY3: [{
			title: "Voltage",
			suffix: "mV"
		}],
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
		yValueFormatString: "###0.00m",
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
//   app.get('/data', function(req, res) {
//     res.sendFile(__dirname + '/sensors.txt');
//   });
});

app.post('/', function(req, res){
	var command = req.body.waterAlert;
	console.log(req.body.waterAlert)
	//res.render('/sensors.html',{ waterAlert: waterAlert});
	res.send(req.body.waterAlert);
})

setInterval(function(){
    io.emit('dataMsg', chartOptions);
}, 1000);

io.on('connection', function(socket){
  io.emit('dataMsg', chartOptions);

  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('/dev/cu.SLAB_USBtoUART', {
  baudRate: 115200
});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialData)

var starttime = null;

function readSerialData(data) {
	data = data.split(",");
	time = parseInt(data[0], 10);
	if (starttime == null) {
		starttime = time;
	}
	time -= starttime;

	steps.push({
		x: time,
		y: parseFloat(data[2])
	});
	battery.push({
		x: time,
		y: parseFloat(data[3])
	});
	temperature.push({
		x: time,
		y: parseFloat(data[4])
	});
	empty.push({
		x:time, 
		y: parseFloat(data[4])
	});
	


}

http.listen(3000, function(){
  console.log('listening on *:3000');
});
