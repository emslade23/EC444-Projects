// code to read data from text file and store it into two arrays.
const csv = require('csv-parser');
const fs = require('fs');

// var StockNames = new Array();
// var StockPrices = new Array();

var amazonPrices = new Array();
var googlePrices = new Array();
var microsoftPrices = new Array();
var facebookPrices = new Array();
var i = 0;
fs.createReadStream('data.csv')
  .pipe(csv())
  .on('data', (row) => {

    console.log(row);
    if (row.Stock == "AMZN"){
      amazonPrices.push(row.Closing);
      //console.log("AMAZON")
    }
    else if (row.Stock == "FB"){
      facebookPrices.push(row.Closing);
      //console.log("fb")
    }
    else if (row.Stock == "GOOGL"){
      googlePrices.push(row.Closing);
      //console.log("goog")
    }
    else if (row.Stock == "MSFT"){
      microsoftPrices.push(row.Closing);
      //console.log("micro")
    }
    // StockNames.push(row.Stock);
    // console.log(StockNames[i]);
    // StockPrices.push(row.Closing);
    // console.log(StockPrices[i]);
    i = i + 1;
  })
  .on('end', () => {
    console.log('CSV file successfully processed');
  });

  var express = require('express'); 
  var app = express();
  
  app.set('view engine', 'ejs');
  
  app.get('/', function(req, res){ 
    res.render('graphs',{ user: "EC444 Staff", amazonPrices: amazonPrices, googlePrices:googlePrices,
    microsoftPrices:microsoftPrices, facebookPrices: facebookPrices, title : "EC444 Group Number 4, Quest 2"});
  });
  
  app.get('/dummygraphs', function(req, res){
    res.render('dummy');
  });
  app.get('/dummygraphs2', function(req, res){
    res.render('dummy2');
  });
  
  
  app.listen(process.env.port || 3000);
