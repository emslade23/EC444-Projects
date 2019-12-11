# Quest 6: Rollup – Capture the Flag Crawler-Style
Authors: Amy Dong, Liz Slade, Quianna Mortimer

2019-12-10

## Summary
The purpose of this quest is to program a crawler to automatically navigate itself though a course and also react to different signals. The crawler must also be able to controlled manually though a web client and also have the functionality to decode a QR code.


## Evaluation Criteria
### Web user interface:
  1. Functional web steering and speed control L,R,F,R, stop  **Completed**
  2. Web display showing historical splits from database **Completed**
  3. Web stream from webcam on crawler **Completed**
  4. All web interfaces in same browser window  **Completed**
  
### Devices:
  1. Functional range sensors **Completed**
  2. Functional collision sensor  **Completed**
  3. Functional PID control on variable speed setpoint  **Completed**
  4. Functional IR sensor **Completed**
  5. Alpha display shows last beacon split time **Completed**
  
### Behaviors:
  1. No collisions in runs **Emily did need to give Herbert a nudge**
  2. Autonomous: drives straight on long walls without touching **Completed**
  3. Autonomous: makes turns, L or R turns around obstacles **Completed**
  4. IR receiver on car receives data from beacon  **Completed**
  5. Car conforms to IR traffic signals (stop and go) **Completed**
  6. Browser or Rpi decodes and displays value of QR code   **Completed**
  7. System records split time to DB  **Completed**
  8. Able to remote drive vehicle **Completed**

## Solution Design
### 1. Hardware
  - ESP - Receive and process all information from the sensors. Sets wheel speed and direction. Server side of the udp connection between the ESP and the Node.js on Raspberry Pi. Sends split time data to Node.js. Receives commands from Node.js for manual driving.
  
  - microLidars - Two of the microLidars are attached to the left side of the crawler. They detect distance between the unit to the wall, and the ESP reads this information by using UART.
  
  - Lidar - Attached to the front of the unit and detects distace between the unit to the wall. ESP reads this information by using i2C.
  
  - Encoder - The optical encoder was used with the black and white circle to detect pulses. Every pulse is 5 cm and multiple it by the time segment, 1sec. The encoder was placed on back wheel so as not to affect by the steering. ESP reads this information by using ADC, and then converts it to wheel speed.
  
  - IR receiver - ESP uses UART to read data from the IR receiver, which will receive data from the traffic light beacons.
  
  - Raspberry Pi - Runs the node.js server and supports the webcam. Receives user input from the web browser and sends commands to the ESP. Processes the QR code. Uses the database leveldb.
  
  - Webcam - Attached to the Raspberry pi, and lives streams to a web browser. Used for capturing the QR code.
  
  - Alphanumeric display - Displays the most recent split time.

### 2. Software
  a. ESP32 code 
  
  - The two microLIDARs were placed at the front and back of one side of the crawler. When detecting a difference in the distance measured in both, the steering servo was turned accordingly. If the back microLIDAR detected less distance, the crawler steers left, if the front detected less, it steered right. The placement of the microLIDARs was made to maintain a parallel relation to one wall.
  
  - The big LIDAR placed at the front end of the crawler is used to detect collisions and also to make the crawler turn right when a wall is approaching.
  
  - The encoder detects the wheel speed and that information is sent to the PID, which adjusts the set speed of the crawler accordingly and maintains a uniform speed.

  - The IR receiver detects the colors sent by the beacons placed on the ground. The ESP processes these color commands and controls the crawler to slowdown, stop or go. When a beacon with a new ID is detected by the IR receiver, the ESP sends this information gets sent to the Node.js server. It also records the elapsed time of this detection and displays that on the alphanumeric display.

  b. Node.js code on Raspberry Pi, web client
  
  - The NodeJS application acts as a client that connects to a server that is running on the ESP32. 
  
  - The checkbox on the application is what initiates switches between autonomous and manual mode. If the check box is unchecked, then the car is in autonomous mode. If the check box is checked, then a manual command gets sent to the ESP32 and processed.
  
  - If the up arrow is pressed, then the command "up" is sent to the ESP32 and the wheels face forward.
  
  - If the left arrow is pressed, then the command "left" is sent to the ESP32 and the wheels turn left. 
  
  - If the right arrow is pressed, then the command "right" is sent to the ESP32 and the wheels turn right. 
  
  - If the down arrow is pressed, then the command "backward" is sent to the ESP32 and the car goes backwards.
  
  - If the go button is pressed, then the command "go" is sent to the ESP32 and the car goes forwards.
  
  - If the stop button is pressed, then the command "stop" is sent to the ESP32 and the car stops. 
  
  - The NodeJS application is also waiting for split times from the ESP32. When it receives split time from the ESP32, it will display it in a table, along with ID, Color, Total Elapsed Time and Run. This data is stored in a leveldb database in a json format (key, value pair). Since there is a socket connection, every time a split time is updated and sent to the NodeJS application, it updates the web client table in real time. 
  
  - We are using a logitech camera



## Sketches and Photos
Figure 1.0: Herbert, the Winner in Our Hearts
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5343.jpeg)

Figure 1.1: Herbert, Q and Amy
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5342.jpeg)

Figure 1.2: Herbert, Q and Shy Amy
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5341.jpeg)

Figure 1.3: A Portrait of Herbert and his Nurturing Mother
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5340.jpeg)

Figure 1.4: Herbert and his Nurturing Mother
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5339.jpeg)

Figure 1.5: Herbert
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5338.jpeg)

Figure 1.6: Herbert 2
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5337.jpeg)

Figure 1.7: Herbert looking at Q for support before his race.
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5335.jpeg)

Figure 1.8: Q giving her son a pep talk.
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5334.jpeg)

Figure 1.9: Herbert meditating
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5333.jpeg)

Figure 1.10: Herbert meditating 2
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5332.jpeg)

Figure 1.11: Herbert and his Dysfunctional Fam
![Herbert](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-6/images/IMG_5345.jpeg)



## Supporting Artifacts
- [Link to repo]()
- [Link to video demo]()


## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
