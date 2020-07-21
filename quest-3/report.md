# Wearable Computer
Authors: Elizabeth Slade, Amy Dong, Quianna Mortimer


October 24, 2019

## Summary
Our Wearable Computer Device allows users to access our web application any where in the world, so long as the application is up and running. We have our ESP32 communicating via UDP protocol to the NodeJS server that is running on Elizabeth's machine. Then, running the server on localhost:3000, allows the user to interact with the web application in the following ways:

In the user input:
- turn water on
- turn water off
- find device
- reset steps

With the checkboxes: 
- disable battery data
- disable temperature data
- disable steps data

Water on and water off is for when a user wants to remind the runner to drink water. If the user inputs "water on", then that signal is sent to the ESP32 which interprets that signal as an indication to flicker the blue LED once every 10 seconds. 

If the user inputs "find device" then that signal is sent to the ESP32 which interprets the signal as an indication to turn the red LED on. 

When a user checks the disable battery, temperature or steps data, in each case, that respective data will go to zero to indicate that the data has been disabled. If the user unchecks that checkbox, then the data collection will be enabled again, and the web application will start reading from the ESP32 once more. 


Furthermore, we have integrated port forwarding such that the user can view our web application from any device connected to the internet.

## Video
- [Link to video demo](https://youtu.be/s8XyCHvXPtk)

## Evaluation Criteria
Objective Criteria
1. Send data across seperate IP network to control remote device through web client- complete
2. Recieve data across seperate IP network from remote sensors into web client- complete
3. Incoporate time and alert scheduling via web client- complete
4. Trigger an immediate alert - complete
5. Web client displays biometric in real time - complete
6. Investigative question reponse: What are steps you can take to make your device and system lower power? 

The major step we can take to make our device and system lower power is to allow the user on the client side to be able to disable certain data collection not just on the graph, but also stop the esp32 from reading the particular sensor data. For example, if the user disables temperature, then the esp32 would stop reading from the thermistor. This would be an effective way to save battery in the long term for the device.

If the user were to disable temperature, we are preventing the esp32 from having to read from a particular gpio pin which would conserve on energy. Furthermore, in order to send the data from esp32 to the server over wifi, this requires a particular power consumption, as high as "Up to 20.5 dBm of transmitting power" (https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf), which we would be saving by disabling reading thermistor data. 

An alternative option is to keep the esp32 in sleep mode until the runner's steps are greater than zero. This would mean that the runner has started moving, and since we are not interested in when the runner hasnt started the race yet, this would be an effective way to conserve power. According to https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/, esp32 active mode consumes 160-260mA, whereas esp32 in light sleep mode consumes 0.8mA. Light sleep mode disables wifi which would be desireable until the runner starts running, then the esp32 could switch to active mode and record the data. 


Bonus: Added an additional command to reset step count. 
 
Qualitative Criteria 
1. Quality of Solution
2. Quality of report.md inlcuding use graphics
3. Quality of code reporting
4. Quality of video presentation

## Solution Design
The Figure 1.1 below is a sketch of our system diagram. It shows the flow and method of communication between the ESP32 and the client. 
The ESP32 chip is connected to the voltage divider, the vibration sensor,  and the the thermitor. 
The ESP32 communicates with the node.js through the Port 3030 using UDP sockets. The Nodes.js in turn uses Port 8080 to communicate with the HTML using the laptop's IP address. The router also forwards the internal 8080 port to Port 5000 and we can remotely access the HTML using DDNS at 128.197.175.240:5000. 
Figure 1._ is a depiction of the client side on canvas.js. The data for each sensor is displayed on one multi-axial graph with the features mentioned in summary. 

## Sketches and Photos
Figure 1.1
![IMG_0666](https://user-images.githubusercontent.com/24261732/67536273-5ef9a800-f6a4-11e9-8d0a-62ebcdadb924.JPG)


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo](https://youtu.be/s8XyCHvXPtk)


## References
class resources: http://whizzer.bu.edu/quests/primary/wearable
class resources: http://whizzer.bu.edu/quests/primary/wearable
http://whizzer.bu.edu/skills/router
http://whizzer.bu.edu/skills/dyndns
https://nodejs.org/api/dgram.html
http://whizzer.bu.edu/skills/wifi
https://en.wikipedia.org/wiki/Network_socket
https://en.wikipedia.org/wiki/Dynamic_DNS
https://nodejs.org/api/dgram.html
https://en.wikipedia.org/wiki/Switch#Contact_bounce
https://www.adafruit.com/product/1766
-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
