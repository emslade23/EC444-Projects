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
Figure 1.1: Finite State Machine
![FSM](https://user-images.githubusercontent.com/24261732/69369318-dd2c7880-0c69-11ea-9867-b4cc6f45a6e6.JPG)

Figure 1.2: Hub and Fobs
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/F4BAE448-A62E-4951-B355-CD5D12906AB7_1_105_c.jpeg)

Figure 1.3: Raspberry Pi
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/DFBCDDAC-0A9F-4EF7-B728-B7CC93FB522F_1_105_c.jpeg)

Figure 1.4: Raspberry Pi Demo of Writing & Reading from Database and Displaying Fob History on Web Client
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/C4AAD81D-3DA0-4C90-8B59-0F8A62FDA791_1_105_c.jpeg)

Figure 1.3: Raspberry Pi
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/DFBCDDAC-0A9F-4EF7-B728-B7CC93FB522F_1_105_c.jpeg)

Figure 1.3: Raspberry Pi
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/DFBCDDAC-0A9F-4EF7-B728-B7CC93FB522F_1_105_c.jpeg)
Figure : Great Job Team 4!
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/E2278929-852B-4645-B21A-C25C4161CAE2_1_105_c.jpeg)

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
