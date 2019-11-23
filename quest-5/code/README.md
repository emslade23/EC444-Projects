# Code Readme

Author: Amy Dong, Elizabeth Slade, Quianna Mortimer
Date: 11/22/2019



In the main folder, hub.c is the code ran on the security hub esp32 and keyfob.c is ran on the key fobs. keyfob.c is altered slightly and contains a unique ID for each of the fobs.

db.is the code that level has created for using levelDB.

mydb folder is the folder that contains our sensor data. We are using a database called LevelDB where the data is stored in files in the repo. 

userinput.js is the node.js file ran on the raspberry pi. It receives information from the security hub and processes it and commands the key fob LED to light up.

table.html displays database data in a table in a webclient.
