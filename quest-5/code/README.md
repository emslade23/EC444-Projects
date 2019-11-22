# Code Readme

Brief explanation on how to navigate your code folder. For example, main consists of the entry function, and lib are where the resource libraries are located.
Skill - Security in Connected System 
1. Identify the weaknesses of node.js to ESP32 system
A few weaknesses of system include the unstable node.js api that forces us to alter the code to be compatible, and the fact that javascript does not do line by line execution which at times requires asyncronous functions. We also used UDP for Quest 3 which sends but does not check if there it's being received. 
2. How can someone attack your system ?
Someone can take control of the local server, flasify the sensor data, or inject code into the user input on web client. 
3. How to overcome those attacks? 
These attacks can be mitigated by having specifc restrictions in the user input on the web client and using TCP instead of UDP. 
4. In the ESP32, the code defines the ip address for communication with the nodes.js server, establishes socket for node.js to ESP32, and configures the the use of the ip address. 
