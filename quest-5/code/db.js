// Author: Amy Dong, Elizabeth Slade, Quianna Mortimer
// Date: 11/22/2019

var level = require('level');
var path = require('path');
var dbPath = path.join(__dirname, 'mydb');
var db = level(dbPath, {
    valueEncoding: 'json'
});
module.exports = db;
