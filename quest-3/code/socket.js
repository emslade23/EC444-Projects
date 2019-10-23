// Required module
var dgram = require('dgram');
const readline = require('readline');

var steps =  new Array();
var battery =  new Array();
var temperature =  new Array();

var answer = "water";

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

// rl.question('Enter command: ', (answer) => {
//   console.log(`You entered: ${answer}`);
//   rl.close();
// });
//
// console.log("answer is");
// console.log(answer);

//var command = prompt("Enter a command:")

// Port and IP
var PORT = 8080;
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
    temperature.push(string[0]);
    battery.push(string[1]);
    steps.push(string[2]);

    console.log(string);
    // Send Ok acknowledgement
    //if (answer !== undefined) {
      //console.log("sending");
      server.send("water",remote.port,remote.address,function(error){
      //if(error){
      //  console.log('MEH!');
      //}
      //else{
        console.log('Sent: Ok!');
      //}
      });
    //};

});

// Bind server to port and IP
server.bind(PORT, HOST);
