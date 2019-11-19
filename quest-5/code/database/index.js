var db = require('./db');
var lineReader = require('readline').createInterface({
    input: require('fs').createReadStream('smoke.txt')
  });
var sensorID1 = {};
var sensorID2 = {};
var sensorID3 = {};
var sensorID4 = {};
var sensorID5 = {};
var sensorBAD = {};
var sensorID;

var time1 = [];
var time2 = [];
var time3 = [];
var time4 = [];
var time5 = [];

var smoke1 = [];
var smoke2 = [];
var smoke3 = [];
var smoke4 = [];
var smoke5 = [];

var temperature1 = [];
var temperature2 = [];
var temperature3 = [];
var temperature4 = [];
var temperature5 = [];

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
                    temperature1.push(parseFloat(line.slice(9, line.length-1)));

                sensorID1 = {
                    time: time1,
                    smoke: smoke1,
                    temperature: temperature1
                };
            break;
            case 2:
                    time2.push(parseInt(line.slice(0,4)));
                    smoke2.push(parseInt(line[7]));
                    temperature2.push(parseFloat(line.slice(9, line.length-1)));
    
                sensorID2 = {
                    time: time2,
                    smoke: smoke2,
                    temperature: temperature2
                };
            break;
            case 3:
                    time3.push(parseInt(line.slice(0,4)));
                    smoke3.push(parseInt(line[7]));
                    temperature3.push(parseFloat(line.slice(9, line.length-1)));

                sensorID3 = {
                        time: time3,
                        smoke: smoke3,
                        temperature: temperature3
                    };  
            break;
            case 4:
                    time4.push(parseInt(line.slice(0,4)));
                    smoke4.push(parseInt(line[7]));
                    temperature4.push(parseFloat(line.slice(9, line.length-1)));

                sensorID4 = {
                        time: time4,
                        smoke: smoke4,
                        temperature: temperature4
                    };  
            break;
            case 5:
                    time5.push(parseInt(line.slice(0,4)));
                    smoke5.push(parseInt(line[7]));
                    temperature5.push(parseFloat(line.slice(9, line.length-1)));

                sensorID5 = {
                        time: time5,
                        smoke: smoke5,
                        temperature: temperature5
                    };   
            break;
            default:
            sensorBAD = {
                time: 5,
                smoke: 5,
                temperature: 5
                }; 
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
    db.put(4, sensorID4);
    db.put(5, sensorID5);

    db.get(1, function(err, value) {  
            if (err) {
            console.error("null");
            }
            else{
                console.log("Sensor 1:", value);
            }
      });
    db.get(2, function(err, value) {  
            if (err) {
            console.error("null");
            }
            else{
                console.log("Sensor 2:", value);
            }
        });
    db.get(3, function(err, value) {  
            if (err) {
            console.error("null");
            }
            else{
                console.log("Sensor 3:", value);
            }
        });
    db.get(4, function(err, value) {  
            if (err) {
            console.error("null");
            }
            else{
                console.log("Sensor 4:", value);
            }
        });
    db.get(5, function(err, value) {  
            if (err) {
            console.error("null");
            }
            else{
                console.log("Sensor 5:", value);
            }
            });
}


