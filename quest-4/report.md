# Quest 4 :Autonomous Driving  - Herbert
Authors: Elizabeth Slade, Amy Dong, Quianna Mortimer

2019-11-08

## Summary
This Quest tasks us with converting a remote controled crawler into an autonomous device that can drive between two endpoints on a road segment. Certain requirements included: 1) Preventing collisions, 2) maintaining driving in the center of the track, 3) maintaining a speed of 0.1 -0.4 m/s, and 4) ending wireless commands to start and stop the crawler.


## Evaluation Criteria
1. Use PID for speed control at 0.1 -0.4 m/s Completed
2. Stops within 10 cm of end Completed
3. Uses PID for holding center of track  Completed
4. Successfully traversesA_B in one position or speed Completed
5. Uses alpha display to show current position or speed Completed
6. Controlled remotely, start and stop  Completed

Investigative Question: Define what you would want in a better sensor for the vehicle. Be very specific in quantifying it’s performance. Please cite materials found on the web to support your response.

The most important thing that I would want in a better sensor for a vehicle is for it to be extremely reliable. With the TFmini Lidars, we discovered that every 10 or so readings, it produced erroneous results. For an autonomous vehicle that would be on the road, it would be essential for it's lidar to be extremely reliable. By reliable, I mean that the sensor should be giving reliable readings at every iteration, 95% of the time. If there is an erroneous result, then we can filter it out by the associated results, but it is essential for the sensor to be reading data as reliably as possible. 

Furthermore, having the sensor be robust is another essential part of the sensor's function. Since the vehicle is moving, it is very likely for the sensor to be moving wildly or for it's hardware to collide with another part of the machine. It is essential, that given this environment, the sensor can still operate reliably.

Furthermore, another important aspect for the ideal sensor would be some type of indicator to let the user know whether or not the sensor is malfunctioning. This would be very important because if the sensor is malfunctioning, the vehicle becomes very dangerous as it will be reading erroneous values from that given sensor. So, if the sensor, assuming it still has power, but is giving erronous values, or isnt wired properly, that it gives a strength signal to denote the quality of the readings that are being read, so that in the software side, we can rank the less strong lidar readings as less valuable in the decision algorithm. I would want the strength reading to be in probabilistic percentages of its accuracy, and i would expect that the strength reading would be accurate + or -5%.

Another very important aspect to the better sensor would be i2c communication. Our tfmini lidars used UART communication which caused problems when we tried to integrate two tfmini sensors communicating via UART. So, if the sensors were communicating via i2c, then we could change their communication address to differentiate the data lines between the two sensors. This woud really help with the hardware configurations as well. 

All in all, a better sensor would be robust to changes in it's environment, would be produce reliable measurements, would communicate over protocol that allows for reliable communication when there are multiple sensors to read from, and lastly, a strength of signal indicator that provides insights into the value of the readings. 

links: https://gisgeography.com/lidar-light-detection-and-ranging/, https://levelfivesupplies.com/sensors-used-in-autonomous-vehicles/


## Solution Design
1. Establish the use of selected sensors on the vehicle
    Wheel speed
       -The optical encoder was used to with the black and white circle to count pulses. Evey pulse is 5 cm and multiple it by the time         segment, 1sec. The encoder was placed on back wheel so as not to affect by the steering.
       -Figure 1.1 is a detailed diagram of the hardware wiring, showing all the main components including the ESP32, ECS, LIDAR, two microLIDARs, alphanumeric display, steering servo, motors, and 7.2v battery.
       -Figure 1.2 is an image of the actual hardware wiring.
    Ranging (IR, LIDAR-LED, LIDAR-Lite, Ultrasonic)
      -For ranging, we used the LIDAR and two microLIDARs.
      -The  LIDAR was coded using i2c while  the mictoLIDARs used uart.
2. Establish control approaches, e.g.,
    Constant distance from walls
    -The two microLIDARs were placed at the front and back of one side of the crawler. When detecting a difference in the distance            measured in both, the steering servo was turned accordingly.
    -If the back LIDAR detected less distance, the crawler steers left, if the front detected less,it steered right.
    Angle of walls
    Orientation of vehicle
    -The placement of the microLIDARs was made to maintain a parallel relation to one wall
    Speed/distance/acceleration of vehicle
3. Establish safety rules, e.g.,
    Stop when detect collision pending
    Stop on wireless command
4. Design software algorithm to satisfy timing of control algorithm, e.g.,
    Accommodate timing limits of sensors (they vary)
    Accommodate actuator timing
    Set PID timing (dt – cycle time)
    - Timer is set to trigger every 100 ms and PID function is called
    Make sure the timing is consistent (e.g., with timers or interrupts)
    Make sure to check for collisions


## Sketches and Photos
Figure 1.1
![IMG_0707](https://user-images.githubusercontent.com/24261732/68505208-ae0f1380-0234-11ea-9dc7-91ff2c7bfeab.JPG)
Figure 1.2
![IMG_0705](https://user-images.githubusercontent.com/24261732/68505173-96378f80-0234-11ea-9137-e66507e776cb.JPG)


## Supporting Artifacts
- [Link to repo](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/tree/master/quest-4)
- [[![Watch the video](https://img.youtube.com/vi/6dlBu9K2hMU/maxresdefault.jpg)](https://youtu.be/6dlBu9K2hMU)

## References
class resources: http://whizzer.bu.edu/skills/lidar-LED
https://cdn.sparkfun.com/assets/5/e/4/7/b/benewake-tfmini-datasheet.pdf
https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/uart.html
https://github.com/PulsedLight3D
http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide#example-circuit
https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/pcnt.html
https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/timer.html

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
