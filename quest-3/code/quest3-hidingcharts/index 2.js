var express = require('express'); 
var app = express();
 
  
  app.set('view engine', 'ejs');
  app.use(express.static(__dirname + '/public'));
  
  app.get('/', function(req, res){ 
    res.render('graphs');
  });
  
  app.get('/dummygraphs', function(req, res){
    res.render('dummy');
  });
  app.get('/dummygraphs2', function(req, res){
    res.render('dummy2');
  });

  app.listen(3000, function(){
    console.log('listening on *:3000');
  });

  
