# Quest 2: Sensor Central 
Authors: Elizabeth Slade, Amy Dong, Quianna Mortimer

2019-10-05

## Summary
This quest required the periodic measuring of IR and ultrsonic sensor in meters and the measuring of thermistor in Celsius which were then plotted in real time on canvas.js. 


## Evaluation Criteria
1. Periodic measuring of ultrasonic sensor in m
2. Periodic measuring of IR sensor in m
3. Periodic measuring temperature in C
4. Results graphs at host
5. Results graphed continuously based on reporting period
6. Investigative questions response


## Solution Design
Sensors
1. Ultrasonic Range Sensor
  -We first changed the ADC width bit size to 10
2. IR range finder
  - We first changed the attentuation to 11, and the bit size to 10. We tested the IR with a distance trail and collected the raw and voltage outputs data for the corresponding distances. Using execl, we plotted the voltage versus the distance measure and created a function to fit the curve.Figure 1.1 shows the three graphs used to show the curve in three states: 1495-2680 mV , 480- 1495mV, and less than 480 mV. 
3. Thermistor
  -The thermistor was one of the two sensors that required a voltage divider with a 220ohm resistor and the thermistor itself acting as a resistor - a capacitor was used to stabalize the voltage. 
4. Battery monitor(Bonus,plotting)
  -The battery monitor was a matter measuring the volatge divider with two resistors, one 16k ohms and the other 220 ohms. 
  
 ESP32 
 - All of the sensors were read using our microprocessor, ESP32. After the ESP32 read the sensors using ADC, the data was then written to the serial port.
 
 NodeJS Application
 - in the NodeJS application, we use the serial port library to read in the sensor data from the serial port. Then, we parse this data, and store it into the associated sensor data arrays. Afterwards, we set up chart options using CanvasJS where we specify the structure and axis and labeling for the graph. We then pass in these chart options into the html page where the charts are rendered and displayed to the client.
 
 Real Time Updating
 - in order to have real time updating, we used the NodeJS library called Socket.io. Using Socket.IO, we create a socket, or a connection between the server and the client, where the server sends the chart options to the client every time more data is added to the chart. As a result, our chart updates in real time. We have a time interval where the chart updates everty 2 seconds. We divide our data into two graphs, the range data and the temperature/battery voltage data.
 
 Investigative Questions: 
 
 

## Sketches and Photos
Figure 1.1
![IMG_1538](https://user-images.githubusercontent.com/24261732/66439077-8d1c8e00-e9fc-11e9-8a9c-28e2132ffe22.JPG)

Figure 1.2


(see photos)


## Supporting Artifacts
- [Link to repo]()
- [![Watch the video](https://img.youtube.com/vi/vtBfmNzGsuc/maxresdefault.jpg)](https://youtu.be/vtBfmNzGsuc)



## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
