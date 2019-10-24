# Code Readme

All of our code is contained in the udp_client folder

- folder "main" is where the c program is located that facilitates UDP communication between the ESP32 and the Nodejs server
- CMakeLists.txt and the makefile is also meant for the c program that is facilitating UDP communication between the ESP32 and the Nodejs server
- node_modules are the installed packages for the nodejs application
- sensors.html is the html page that gets displayed to the user through the browser and is how the client can communicate with the server
- userinput.js is the javascript file that facilitates the routing and the rendering of the sensors.html page, it is also responsible for reading from the serial port and initializing the graphs, and setting up a socketio connection with the client
