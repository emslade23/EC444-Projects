# Quest 5: Secure Key
Authors: Amy Dong, Liz Slade, Quianna Mortimer

2019-11-22

## Summary
This quest tasks us with creating a key fob using a the esp32 board, a transmitter, and a receiver. This key is met to interact with a hub by sending a unique ID and code which the hub then relays to a servers along with its own ID. Through a local network, the server on the RPi and a web client receive both IDs, the location and time of the interaction as well as the person. The hub then sends an unlocked signal to the fob triggering a green LED on the board. Figure 1.1 below is an illustration of a state machine and chart for the overall system.  


## Evaluation Criteria
1. Fob relays {fob_ID,code} to security hub, hub sends {fob_ID,hub_ID,code} to server;server responds to fob, fob green light turns on Completed
2. Logs fob accesses to database {fob_ID,hub_ID,person,time,location} Completed
3. Database is on RPi Completed
4. Web-based management interface shows real-time active unlocked fobs and history of unlocked fobs (with actual time of unlock) Completed
5. Uses at least 3 fobs with unique IDs Completed
6. Demo delivered at scheduled time and report submitted in team folder with all required components Completed
Bonus: We included DDNS to allow access to the web client on multiple devices if connect to an service.  

**Investigative question: comment on the security of your system. How would you best hack into this system if you were so inclined? How could you prevent this attack? Describe the steps.**


  When thinking about the security of our system, there are three main parts: the security of the receiver/transmitter communication, the security of the UDP socket communication, and the security of the web client.

  #### Security of the Receiver and Transmitter Communication
  A major vulnerability in this communication is that anyone can create a receiver which can receive the transmitted fob signal. If a bad actor makes his own receiver and receives the fob signal, then the bad actor could potentially decode the password for the fob. If this fob was meant to unlock a car or a house, then the bad actor could program another fob to emit the same password signal and trick the receiver to unlock the car or house. This presents a huge security threat with RX/TX communication. One solution would be encrypting the data with SHA256 encryption so that it would be nearly imopssible to decrypt the data without the hash function.

 Another solution could be to have the transmitter and receiver engage in some type of a handshake to confirm the recipient, before any data is sent, could also be an effective way to keep the data in the hands of the people authorized to have it.

  #### Security of the UDP Socket Communication
 A big vulnerability in the UDP Socket Communication between the ESP32 and the NodeJS app is the mere fact of using UDP. Since there is no acknowledgement, meaning that there is no way to know if the person sending to the NodeJS server is authorized to be doing so, this means that anyone can be sending larges volumes of packets. This could potentially result in a Denial of Service attack preventing the people who are suppose to be viewing the data from seeing it. A solution for this vulnerability would be to have some code that monitors packets coming into the server, and has a certain threshold of packets per second, and when that threshold is reached, then maybe it doesnt accept packets for some time, or it sends a warning to an admin.

 Furthermore, another vulnerability that comes with UDP communication is the ease of packet sniffing. It would be relatively simple to intercept a packet being sent from the ESP32 to the NodeJS server, decode the message, and then understand the data being sent. This vulnerability could be prevented by switching from UDP communication where there is no SYM/ACK talking, to TCP communication where there is a handshake before any real data is transmitted. This handshake allows for an acknowledgement that the client, or the person receiving the data, is in fact authorized to receive the data.



## Solution Design
1. Hardware: IR TX/RX
  a. Fob keys - Three key fobs were built (one for each team member) using an ESP32 board, one IR receiver diode for receiveing information, one IR LED used to transmit information, one button to trigger the emitter, and one green LED triggered when the unlock was successful. The fob IDs range from 1 to 3 which were assigned to one person in the group. Fob 1 belongs to Amy, 2 to Quianna, and 3 for Lizzy.
  b. Hub - Due to ESP32 board shortages, one hub was physically built, however  two IDs were assigned to it - a button was used to switch between the two IDs. The hub is essentially built the same as the fob ( with a IR  diode and IR LED) with the exception of the use of the LED. On the hub, the LED lights up when it successfully received information. The hub IDs were each assigned to a place. Hub 1 is the "Front door" while hub 2 is the "Garage door".
  c. RPi - This quest required the node.js to be on the RPi for authentification and the display on the web client.
2. Software

  a. ESP32 Code - For the ESP 32 there are two different kinds of code, one is for the key fob, one is for the security hub. For the key fob code, it sends its unique ID and the passcode to the security hub using Uart and also a socket is established between the key fob and the server on raspberry pi. Whenever the button is pressed, the Uart and the UDP client start to function for a brief time, so that the transmission and sockets are not functioning continuously but only when the button is pushed. When the button is pressed the fob sends and receives information from the raspberry pi only once, and the received information contains a command that will trigger it’s green LED to light up.

  For the security hub, it continuously reads the transmissions through Uart, and if the transmission matches A certain structure, it would then create a socket and pass The identification information to the raspberry pi. The udp client socket is not transmitting data continuously so that the server is able to only log the activity once.



  b. Node.js and Web Client - The NodeJS application reads in timestamp, fob id, location, and hub id data from the ESP32 through UDP socket. It then stores this data into a json format in the LevelDB database. The database is set up as follows:

          LevelDB Structure - Key : Value

           {

                time: [ array of timestamps ],
                sensor: [array of sensor ID 1],
                hub: [array of hub references for each fob 1 request],
                name: [array of names of the owner of this fob],
                location: [array of locations based on the hub reference]

           }

  This is the case for all of the sensors. We have the key stored as the sensor id number and the value as a json that keeps getting concatenated with new values as the fobs are used.

  We then pull data from the leveldb database for each of the fobs. Then we send this fob data for each of the fobs, to the client side of the app through a UDP socket. Using the Socket.IO library that we installed using npm, we are able to update the tables with fob data in real time. We have the client successfully running on a Raspberry Pi that is connected to our Group 4 wifi. Because the NodeJS web application is running on the RasPi, we had to be sure to change the LAN IP from our Laptop's IP to the RasPi IP so that the Fobs and the Hub can communicate with the RasPi. It is important to note here that because our NodeJS application is running on the RasPI, that means our LevelDB database is also running on the RasPi. In order to make this step work, it required a lot of debugging and we figured out that using nvm install 9, updates nodejs and allows for an easy install of level (the package for leveldb).



Technical Design
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/01ECDD7D-85B0-449C-8C8A-A67A434BA551_1_105_c.jpeg)

## Sketches and Photos
Figure 1.1
![IMG_0722](https://user-images.githubusercontent.com/24261732/69471848-0fc19880-0d72-11ea-826d-d97d481f9590.JPG)
![fsm_table](https://user-images.githubusercontent.com/24261732/69471882-5a431500-0d72-11ea-9558-9c2a70e335e4.png)

 Figure 1.2: Hub and Fobs
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/F4BAE448-A62E-4951-B355-CD5D12906AB7_1_105_c.jpeg)

Figure 1.3: Raspberry Pi
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/DFBCDDAC-0A9F-4EF7-B728-B7CC93FB522F_1_105_c.jpeg)

Figure 1.4: Raspberry Pi Demo of Writing & Reading from Database and Displaying Fob History on Web Client
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/C4AAD81D-3DA0-4C90-8B59-0F8A62FDA791_1_105_c.jpeg)

Figure 1.5: Fob and Hub
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/A9830081-BF52-4230-A55D-B488FDE25ED1_1_105_c.jpeg)

Figure 1.6: Data on Raspi and Web Client
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/A08A55CC-35A0-426D-A9F8-32E90C204F24_1_105_c.jpeg)

Figure 1.7: Display
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/9FCC4B36-4057-48C0-A009-348309C6EF78_1_105_c.jpeg)

Figure 1.8: Green Light!
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/90E4CB06-A3FE-448E-855D-0B0419FECB8D_1_105_c.jpeg)

Figure 1.9: Amy's Fob
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/7AA7F0B0-BD05-42AA-8277-145064782EB9_1_105_c.jpeg)

Figure 1.10: The Hub
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/6D69EA30-2933-489D-A513-88FBA7BD5F4F_1_105_c.jpeg)

Figure 1.11: Fob
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/6830EEE9-B223-4C3F-9B4C-3D1EDEE8708A_1_105_c.jpeg)

Figure 1.12: Data on Raspi and Web Client
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/A08A55CC-35A0-426D-A9F8-32E90C204F24_1_105_c.jpeg)

Figure 1.13: Hub
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/534AD0E8-32E7-44C0-A6E4-67C9467C9CFA_1_105_c.jpeg)

Figure 1.14: Sensor Setup
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/4A484D44-1776-4E25-BFE1-0CC25FB58F77_1_105_c.jpeg)

Figure 1.15: Web Client
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/18EECF87-63C5-4839-BFDA-567C04D23605_1_105_c.jpeg)

Figure 1.16: Professor Little
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/1875AD8E-1367-4885-8986-988AF44B8775_1_105_c.jpeg)

Figure 1.17: Raspi on Device List
![RasPi](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/0A4E7A60-A7FC-4910-81D5-54BD1DEBC1BE_1_102_o.jpeg)

Figure 1.18: Great Job Team 4!
![Circuit](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/blob/master/quest-5/images/E2278929-852B-4645-B21A-C25C4161CAE2_1_105_c.jpeg)


## Supporting Artifacts
- [Link to repo]()
- [![Watch the video](https://img.youtube.com/vi/rQ8CEyopQT4/maxresdefault.jpg)](https://youtu.be/rQ8CEyopQT4)


## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
