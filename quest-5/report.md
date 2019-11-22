# Quest 5: Secure Key 
Authors: Amy Dong, Liz Slade, Quianna Mortimer

2019-11-22

## Summary
This quest tasks us with creating a key fob using a the esp32 board, a transmitter, and a receiver. This key is met to interact with a hub by sending a unique ID and code which the hub then relays to a servers along with its own ID. Through a local network, the server on the RPi and a web client receive both IDs, the location and time of the interaction as well as the person. The hub then sends an unlocked signal to the fob triggering a green LED on the board. Figure 1.1 below is an illustration of a state machine for the overall system.  


## Evaluation Criteria
1. Fob relays {fob_ID,code} to security hub, hub sends {fob_ID,hub_ID,code} to server;server responds to fob, fob green light turns on Completed 
2. Logs fob accesses to database {fob_ID,hub_ID,person,time,location} Completed 
3. Database is on RPi Completed 
4. Web-based management interface shows real-time active unlocked fobs and history of unlocked fobs (with actual time of unlock) Completed 
5. Uses at least 3 fobs with unique IDs Completed 
6. Demo delivered at scheduled time and report submitted in team folder with all required components Completed 
7. Investigative question: comment on the security of your system. How would you best hack into this system if you were so inclined? How could you prevent this attack? Describe the steps.

8. Bonus: We included DDNS to allow access to the web client on multiple devices if connect to an service.  

## Solution Design
1. Hardware: IR TX/RX 
  a. Fob keys - Three key fobs were built (one for each team member) using an ESP32 board, one IR receiver diode for receiveing information, one IR LED used to transmit information, one button to trigger the emitter, and one green LED triggered when the unlock was successful. The fob IDs range from 1 to 3 which were assigned to one person in the group. Fob 1 belongs to Amy, 2 to Quianna, and 3 for Lizzy. 
  b. Hub - Due to ESP32 board shortages, one hub was physically built, however  two IDs were assigned to it - a button was used to switch between the two IDs. The hub is essentially built the same as the fob ( with a IR  diode and IR LED) with the exception of the use of the LED. On the hub, the LED lights up when it successfully received information. The hub IDs were each assigned to a place. Hub 1 is the "Front door" while hub 2 is the "Garage door". 
  c. RPi - This quest required the node.js to be on the RPi for authentification and the display on the web client. 
2. Software
  a. ESP32 code
  b. Node.js 
  c. Web client 
3. Combined system  


## Sketches and Photos
Figure 1.1
![FSM](https://user-images.githubusercontent.com/24261732/69369318-dd2c7880-0c69-11ea-9867-b4cc6f45a6e6.JPG)

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
