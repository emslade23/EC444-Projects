
=======
# Retro Alarm Clock!
Authors: Amy Dong, Quianna Mortimer, Elizabeth Slade

2019-09-23

## Summary

## What's left?
1.) get servo to work for 180, seconds and minutes
2.) up and down RTOS skill
3.) quest
=======
In this quest, we made a Retro Alarm Clock. The primary function of the alarm clock is to work as a timer. So when a user inputs a specific time, then our clock will take in that time and start a timer. When this timer starts, the seconds will be recorded by one of the servos. Then, the minutes will be recorded by the last two digits on the Alphanumeric display and by the second servo. The hours will be recorded by the first two digits of the alphanumeric display.

## Design Specifications and Evaluation Criteria:
### Objective Criteria
1. Two servos indicate actual time in seconds and minutes: **Yes**
2. Alphanumeric display indicates actual time in hours and minutes: **Yes**
3. Can set time: **Yes**
4. Demo delivered at scheduled time and report submitted in team folder with all required **Yes**
5. Investigative question response: How can you synchronize multiple ESP clocks with each other? You can synchronize multiple ESP clocks using FreeRTOS. In our project, we are creating one task for the alphanumeric display, one task for the seconds servo to keep track of seconds passing, and one task for minutes servo to keep track of minutes passing (see below the code)


'''


	void init() {				// Convenient way to organize initialization
	     ...    				// Do it in this sub

	}

	static void task_1()			// Define your first task here

	{
		while(1){			// Or for( ;; )

			Do some stuff;		// Your task code here

		}
	}

	static void task_2()			// Define your second task here

	{

		while(1){			// Or for( ;; )

			Do some stuff;		// Your task code here

		}

	}

	voide app_main()
	{
		init();				// Initialize stuff

		xTaskCreate(task_1, "task_1",1024*2, NULL, configMAX_PRIORITIES, NULL);

		xTaskCreate(task_2, "task_2",1024*2, NULL, configMAX_PRIORITIES-1, NULL);

	} // Instantiate tasks with priorites and stack size

'''

(from ec444 site)

Here, you can perform multiple tasks at different speeds and have them run in parallel. This is useful because it allows the ESP clocks to synchronize, and not cause unwanted results between different processes.


We used delays to keep track of the time in our alarm clock. The problem with delays is that they are not completely accurate, so over time, the minutes and seconds will become less and less accurate to reality. 
If you want to constantly synchronize our retro alarm clock with the actual time and keep making sure that the delay is not causing noise in the system, then it makes the most sense to use an interrupt where you are constantly checking the alarm clock's time with the system time in the computer or in the ESP-IDF machine. This way, you are constantly resetting the alarm such that it stays updated with the current time. This removes the noise from the system. 
****

### Qualitative Criteria
1. Quality of solution
2. Quality of report.md including use of graphics
3. Quality of code reporting
4. Quality of video presentation



## Solution Design
Any specific details about your approach, how you did it, components, or items to call out that make your work special
- We first completed the LED binary incrementing skill where the lights go from 0 to 15 and back to 0.
- Then, we completed the console IO skill. In this skill, we had some roadblocks with UART, so we ended up using gets(buf) to take in user input and have it be read by ESP-IDF. This was much more effective for us and we were all able to accomplish the echo task, and the toggle task, and the hex conversion task. This was very helpful for the quest because we learned about console-io and how to take in user input and use it in our programs. This part was particularly helpful because for the quest we had to take in user input for the hours and minutes to set the timer.
- After this task, we also spend time working on the alphanumeric display. This was very helpful for the quest because we needed to understand how this display works in order to complete the display task for the quest. This task helped us to discover the font table map and the structure of how to display these elements to the screen. We used these functions to display the time on the display for our quest.
- We also learned a lot about pulse width and degrees from the servo skill. We particularly learned how each servo has a different pulse width and you have to increase the different between max pulse width - min pulse width in order to make each second take up a more visible angle along the 180 degree arc. We also had to increase the counter by three for every iteration in order to allow for a 180 degree angle to take 60 seconds.
- With the RTOS skill, we learned about running multiple processes, with different clock speeds at once. This was very helpful in showing us the syntax of how it should work. We used RTOS in order to have the Alphanumeric Display count the minutes, have the seconds servo count up to 60 seconds, and the minutes servo increment every minute.
- Special Elements
	- In order to create the seconds ticker, we created a xTaskDelay of 100 milliseconds in order for it to delay every second.
	- In order to create the minutes ticker, we created a xTaskDelay of 6000 milliseconds in order to delay until a minute passes.
	- in order to increment the numbers for the hours and the minutes, we have counters to keep track of this, and also delays so that each iteration takes a second, so that 60 iterations equals one minute passing and 3600 iterations equals one hour passing.

## Sketches and Photos
It is the time 2:09 and so as you can see the second display is ticking and the minutes display is aligned properly to 9 minutes given that our servos range is 180 degrees.
<center><img src="./images/IMG_0130.jpg" width="70%" /></center>

You can see that the angle of rotation increments by three each time, because since our servos range is 180 degrees, we have 3 degrees of rotation occuring per second.
<center><img src="./images/IMG_0761.jpg" width="70%" /></center>

This is an example at 1:32 and you can see that the seconds servo is ticking away, and the minutes servo is aligned with approximately 30 minutes.
<center><img src="./images/IMG_7307.jpg" width="70%" /></center>
<center>

</center>


## Supporting Artifacts
- [Link to repo](https://github.com/BU-EC444/Team4-Dong-Mortimer-Slade/edit/master/quest-1/report.md)
[![Watch the video](https://img.youtube.com/vi/3sD5_wn7ulk/maxresdefault.jpg)](https://youtu.be/3sD5_wn7ulk)



## References
References or links to any code or techniques adopted if not your own
- We used the EC444 site for code throughout the project. Specifically the code for Console IO, Alphanumeric Display, Servos, FreeRTOS, and some more
- We also referenced the esp-idf example documentation: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_basic_config/main/mcpwm_basic_config_example.c
-----

