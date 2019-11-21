# Quest 5: Secure Key 
Authors: Amy Dong, Liz Slade, Quianna Mortimer

2019-11-22

## Summary
This quest tasks us with creating a key fob using a the esp32 board, a transmitter, and a receiver. This key is met to interact with a hub by sending a unique ID and code which the hub then relays to a servers along with its own ID. Through a local network, the server on the RPi and a web client receive both IDs, the location and time of the interaction as well as the person. The hub then sends an unlocked signal to the fob triggering a green LED on the board. Figure 1.1 below is an illustration of a state machine for the overall system.  


## Evaluation Criteria
1. Fob relays {fob_ID,code} to security hub, hub sends {fob_ID,hub_ID,code} to server;server responds to fob, fob green light turns on
2. Logs fob accesses to database {fob_ID,hub_ID,person,time,location}
3. Database is on RPi
4. Web-based management interface shows real-time active unlocked fobs and history of unlocked fobs (with actual time of unlock)
5. Uses at least 3 fobs with unique IDs
6. Demo delivered at scheduled time and report submitted in team folder with all required components
7. Investigative question: comment on the security of your system. How would you best hack into this system if you were so inclined? How could you prevent this attack? Describe the steps.

## Solution Design



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
