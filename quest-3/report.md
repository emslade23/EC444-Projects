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
Objective Criteria
1. Send data across seperate IP network to control remote device through web client- complete
2. Recieve data across seperate IP network from remote sensors into web client- complete
3. Incoporate time and alert scheduling via web client- complete
4. Trigger an immediate alert - complete
5. Web client displays biometric in real time - complete
6. Investigative question reponse: What are steps you can take to make your device and system lower power? 


Bonus: Added an additional command to reset step count. 
 
Qualitative Criteria 
1. Quality of Solution
2. Quality of report.md inlcuding use graphics
3. Quality of code reporting
4. Quality of video presentation

## Solution Design



## Sketches and Photos
Figure 1.1
![IMG_0666](https://user-images.githubusercontent.com/24261732/67536273-5ef9a800-f6a4-11e9-8d0a-62ebcdadb924.JPG)


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo](https://youtu.be/s8XyCHvXPtk)



## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
