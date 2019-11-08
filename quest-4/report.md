# Quest 4 :Autonomous Driving  - Herbert 
Authors: Elizabeth Slade, Amy Dong, Quianna Mortimer

2019-11-08

## Summary
This Quest tasks us with converting a remote controled crawler into an autonomous device that can drive between two endpoints on a road segment. Certain requirements included: 1) Preventing collisions, 2) maintaining driving in the center of the track, 3) maintaining a speed of 0.1 m/s, and 4) ending wireless commands to start and stop the crawler. 


## Evaluation Criteria 
1. Use PID for speed control at 0.1 -0.40 m/s Completed
2. Stops within 10 cm of end Completed 
3. Uses PID for holding center of track  Completed
4. Successfully traversesA_B in one position or speed Completed
5. Uses alpha display to show current position or speed Completed 
6. Controlled remotely, start and stop  Completed
7. Investigative Question: Define what you would want in a better sensor for the vehicle. Be very specific in quantifying it’s performance. Please cite materials found on the web to support your response.


## Solution Design
1. Establish the use of selected sensors on the vehicle
    Wheel speed
       -The optical encoder was used to with the black and white circle to count pulses. Evey pulse is 5 cm and multiple it by the time         segment, 1sec. The encoder was placed on back wheel so as not to affect by the steering. 
    Ranging (IR, LIDAR-LED, LIDAR-Lite, Ultrasonic)
      -For ranging, we used the LIDAR and two microLIDARs.
      -The  LIDAR was coded using i2c while  the mictoLIDARs used uart. 
2. Establish control approaches, e.g.,
    Constant distance from walls
    -The two microLIDARs were placed at the front and back of one side of the crawler. When detecting a difference in the distance            measured in both, the steering servo was turned accordingly. 
    -If the back LIDAR detected less distance, the crawler steers left, if the front detected less,it steered right. 
    Angle of walls
    Orientation of vehicle
    -The placement of the microLIDARs was made to maintain a parallel relation to 
    Speed/distance/acceleration of vehicle
3. Establish safety rules, e.g.,
    Stop when detect collision pending
    Stop on wireless command
4. Design software algorithm to satisfy timing of control algorithm, e.g.,
    Accommodate timing limits of sensors (they vary)
    Accommodate actuator timing
    Set PID timing (dt – cycle time)
    Make sure the timing is consistent (e.g., with timers or interrupts)
    Make sure to check for collisions


## Sketches and Photos
<center></center>  
<center> </center>


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
