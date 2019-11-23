// Author: Amy Dong, Elizabeth Slade, Quianna Mortimer
// Date: 11/22/2019

// import libraries
var level = require('level');
var path = require('path');
// create a database folder to store data
var dbPath = path.join(__dirname, 'mydb');
// store values that are jsons
var db = level(dbPath, {
    valueEncoding: 'json'
});
// export database
module.exports = db;
