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
## Evaluation Criteria



## Solution Design



## Sketches and Photos
<center><img src="./images/example.png" width="70%" /></center>  
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
