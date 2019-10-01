var http = require('http');
var fs = require('fs');
http.createServer(function (req, res) {
  fs.readFile('CanvasJS-Multi-Series-Chart.html', function(err, data) { //change the name of the file with different html graphs
    res.writeHead(200, {'Content-Type': 'text/html'});
    res.write(data);
    res.end();
  });
}).listen(8080);

//PUT THE CANVASJS HTML FILE IN THE SAME FOLDER
