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
var temp_checkbox = false;
var battery_checkbox = false; 
var step_checkbox = false; 
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
	axisY: [{
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

});
var command = "you haven't entered a command yet";
app.post('/', function(req, res){
	var command = req.body.command;
	console.log(req.body.command)
	res.sendFile(__dirname + '/sensors.html', {command:command});
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

//get data
io.on('connection', function(socket){
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


//serial port
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('/dev/cu.SLAB_USBtoUART', {
  baudRate: 115200
});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialData)

var starttime = null;
var zero_num = 0; 

function readSerialData(data) {
	data = data.split(",");
	time = parseInt(data[0], 10);
	if (starttime == null) {
		starttime = time;
	}
	time -= starttime;

	if(step_checkbox == false){  
		steps.push({
			x: time,
			y: parseFloat(data[2])
		});
	}else{
		steps.push({
			x: time,
			y: zero_num
		});	
	}

	if(battery_checkbox == false){  
		battery.push({
			x: time,
			y: parseFloat(data[3])
		});
	}else{
		battery.push({
			x: time,
			y: zero_num
		});
	}

	if(temp_checkbox == false){  
		temperature.push({
			x: time,
			y: parseFloat(data[4])
		});
	}else{
		temperature.push({
			x: time,
			y: zero_num
		});
	}
	empty.push({
		x:time, 
		y: parseFloat(data[4])
	});
	


}

http.listen(3000, function(){
  console.log('listening on *:3000');
});
