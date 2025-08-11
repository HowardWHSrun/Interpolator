
8/8/25
The one I find most helpful for understanding what's going on is the zoomed in view.

One thing that confuses me on this figure is the lower bound. It seems to show that the vehicle decelerates, stops, and then instantly hits max speed toward the end. In other words, that almost 90 degree bend in the bottom right corner of each shaded region seems to imply a higher than expected acceleration. There are some sharp corners in the upper bounds of this figure too which also seem to show instant speed change.

To help see what is going on I suggest creating a new figure that shows distance vs time and speed vs time as two vertically stacked subplots. I've attached an example of what it might look like for the first interval in your zoomed in plot.

8/6/25
1. what is the audience for this interpolator 
2. what do users want to see? 

Maximum speed	60 mph (97 km/h)
Weight	76,000–97,328 lb (34,473–44,147 kg)
Steep gradient	10%
Power output	696–776 hp (519–579 kW)
Acceleration	3.0 mph/s (1.3 m/s2) maximum
Deceleration	5.0 mph/s (2.2 m/s2) maximum

8/3/25
As discussed during our meeting today, we would like you to start working on a GTFS interpolator . The system will have two inputs: 
1. GTFS-realtime data that we have collected that shows vehicle positions, speeds, and timestamps
2. All the constants we know about the system such as track location for rail vehicles, vehicle weight, max acceleration, max speed, observed speed profiles, etc.

The output should be our best estimate for the speed and location of each vehicle at any point in time as well as some measure of how certain we are. There are various ways to present this, it could be only the most likely path, it could be some probability density function for each timestamp, it could be the set of all possible trajectories given the constraints, etc.

I've attached some data for you to start looking at from the SFMTA K Line in San Francisco. I've also attached a figure showing what some of the data looks like. Please focus on working with the data before about 27,000 ft from the start. The data before this point is spaced out by about 15 seconds, after that point the update frequency drops to 60 seconds which will make interpolation harder.

According to Wikipedia this is one of the vehicles that runs on the K Line: https://en.wikipedia.org/wiki/Siemens_S200. This shows the max accel, max deccel, max speed, weight, and a couple of other things.

Please do your exploratory data analysis to get a feel for the data, see what it contains, try plotting speed vs dist, speed vs time, dist vs time. Then propose how you plan to approach this interpolation task. so we can discuss it before you get started.
