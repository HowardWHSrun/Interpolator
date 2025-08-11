








8/7
David: 
To start with, add these assumptions:
*  No reverse travel
*  Stop at station is between 10 - 40 s
*  Stop at intersection is between 0 - 90 s

Temba: 
For the portion of data that you're looking at before 25000 ft or so, all the stops are at underground stations and none are at intersections. I can share the distance values for these stations that I came up with on Monday, but for the time being I would suggest plotting speed vs distance for 20 or so vehicles, and it should be very apparent where the stations are as almost all vehicles stop at almost all stations.

Here's a summary of all the constraints you and David have identified:

1. Initial velocity condition:
At the initial time t_0, the velocity v(t_0) is equal to v_0.

2. Final velocity condition:
At the final time t_1, the velocity v(t_1) is equal to v_1.

3. Speed constraint (velocity bounds):
For all times t between t_0 and t_1, the velocity must remain between a minimum speed v_{\text{min}} and a maximum speed v_{\text{max}}.
That is: v_{\text{min}} \le v(t) \le v_{\text{max}} for all t \in [t_0, t_1].

4. Acceleration constraint (bounded derivative of velocity):
For all times t between t_0 and t_1, the rate of change of velocity (acceleration) must be between -a_{\text{max}} and a_{\text{max}}.
That is: -a_{\text{max}} \le \frac{dv}{dt} \le a_{\text{max}} for all t \in [t_0, t_1].

5. Displacement constraint (integral of velocity equals total displacement):
The integral of the velocity from t_0 to t_1 must equal the change in position s_1 - s_0.
That is: \int_{t_0}^{t_1} v(t) \, dt = s_1 - s_0.

Where:
- v_min = 0 mph (no reverse allowed)
- v_max = 60 mph
- a_min = -5 mph/s (braking)
- a_max = 3 mph/s (accel limit)

Assumptions for the "likely" visualization profile (not hard constraints):
- Symmetric accel/decel = 2 mph/s (no reverse; nonnegative speeds)
- If both endpoints are near-stopped:
  - Short moves (≤ ~80 ft, ≈ one car length): allow 0–90 s dwell (intersection-like)
  - Otherwise: prefer 10–40 s dwell (station-like), if slack permits
- The likely curve uses a minimal-time triangular/trapezoidal velocity profile capped at v_max with 2 mph/s, allocating slack to dwell then cruise.

And t0, v0, s0, t1, v1, s1, come from a pair of consecutive data points.

One way to get a smooth path in between the two data points is to minimize the sudden changes in acceleration. I looked this up and it is apparently called the "Minimum Jerk Trajectory". It won't tell you where the vehicle stopped or for how long, but if the vehicle was moving the whole time it will give a smooth motion in between.


8/4
As discussed during our meeting today, we would like you to start working on a GTFS interpolator . The system will have two inputs: 
1. GTFS-realtime data that we have collected that shows vehicle positions, speeds, and timestamps
2. All the constants we know about the system such as track location for rail vehicles, vehicle weight, max acceleration, max speed, observed speed profiles, etc.

The output should be our best estimate for the speed and location of each vehicle at any point in time as well as some measure of how certain we are. There are various ways to present this, it could be only the most likely path, it could be some probability density function for each timestamp, it could be the set of all possible trajectories given the constraints, etc.

I've attached some data for you to start looking at from the SFMTA K Line in San Francisco. I've also attached a figure showing what some of the data looks like. Please focus on working with the data before about 27,000 ft from the start. The data before this point is spaced out by about 15 seconds, after that point the update frequency drops to 60 seconds which will make interpolation harder.

According to Wikipedia this is one of the vehicles that runs on the K Line: https://en.wikipedia.org/wiki/Siemens_S200. This shows the max accel, max deccel, max speed, weight, and a couple of other things.

Please do your exploratory data analysis to get a feel for the data, see what it contains, try plotting speed vs dist, speed vs time, dist vs time. Then propose how you plan to approach this interpolation task. so we can discuss it before you get started.


