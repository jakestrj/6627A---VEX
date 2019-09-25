Motion Profiling, Tracking, and Pursuit via Odometry Theory on Differential Systems
====================================================================
## Application in Robotics 
---

Jake R. Johnson
---------------

## Abstract
This paper defines not only control theory for 4 wheel fixed-differential drivebases in VEX/FRC, but in any general application. Anywhere precise drivebase sensors are located, odometric measurements (absolute), velocity, and acceleration can be determined and used to construct movement profiles. Vision tracking and LIDAR are rigorous alternatives to Odometry, however they require more extensive tuning and difficult integration with motion profiles. Documented here is are most applicable and innovative processes for achieving desired movement in any system.

Many of these implementations were over the course of program development in team 6627A, during my time as head programmer and captain. 

---
## Odometry
Odometry theory is more generally defined as high-level position tracking
> Basic implementation, relative to start pos. (0,0)
> - Field-relative rather than bot-relative, reducing accumulated errors. To increase precision, it is best to minimize precise stops and merge movements together (discussed later as `pure pursuit control`) and follow paths

Model is defined as: <br>
Movement from: <img src="svgs/b0abba107ce2fbdfb33d6bdade88770c.gif?invert_in_darkmode" align=middle width=145.92830504999998pt height=27.94539330000001pt/> <br>
Odometry telemetry: <img src="svgs/9117cefc58ccce8de8d98cc3ed3e81db.gif?invert_in_darkmode" align=middle width=84.9428217pt height=24.65753399999998pt/> (absolute at time <img src="svgs/4f4f4e395762a3af4575de74c019ebb5.gif?invert_in_darkmode" align=middle width=5.936097749999991pt height=20.221802699999984pt/>)

### Modeling Motion
(1) Change in average radius, in terms of inner and outer radius, is used to calculate change in center of 4 wheel system<br>

<p align="center"><img src="svgs/68ff04b336caad98348dff366d388624.gif?invert_in_darkmode" align=middle width=120.51887264999999pt height=33.62942055pt/></p>  

<br>

<p align="center"><img src="svgs/ca8b8db165bd910a0b255999546ccc25.gif?invert_in_darkmode" align=middle width=221.51480339999998pt height=49.315569599999996pt/></p>

<br>
(2) Change in theta moving direction as a function of change in inner radius, represents clear advantage over gyroscopic sensors in taking absolute theta<br>


<p align="center"><img src="svgs/aa1c9e9f33b14a857f833028a96df7f6.gif?invert_in_darkmode" align=middle width=133.77235905pt height=34.7253258pt/></p> 

<br>

<p align="center"><img src="svgs/9f925880645d55342c7ecf25991112dc.gif?invert_in_darkmode" align=middle width=222.88463339999998pt height=69.0417981pt/></p>

<img src="./media/img2.png" width="300"> <img src="./media/img3.png" width="384">

These values can be used to create a vector representing absolute position difference (delta x, delta y, delta theta) between last state (assuming non-steady state)

<p align="center"><img src="svgs/1af53c69bdd55402a7c86eab81b0bb2e.gif?invert_in_darkmode" align=middle width=171.30320129999998pt height=16.438356pt/></p> <br>

<p align="center"><img src="svgs/c4ba56e240ec4d771f9777823e98808b.gif?invert_in_darkmode" align=middle width=143.4706779pt height=16.438356pt/></p> <br>

<br>

<p align="center"><img src="svgs/8bdd5bc05c4b563d0287f3668d10c30e.gif?invert_in_darkmode" align=middle width=171.00566834999998pt height=16.438356pt/></p> <br>

<p align="center"><img src="svgs/c4ba56e240ec4d771f9777823e98808b.gif?invert_in_darkmode" align=middle width=143.4706779pt height=16.438356pt/></p>

<br>

<p align="center"><img src="svgs/440e767aa26710959ed21ef3ece2f4e2.gif?invert_in_darkmode" align=middle width=408.2217282pt height=59.1786591pt/></p>
<p align="center"><img src="svgs/e13556822bb81c69fa2d9c2473794f17.gif?invert_in_darkmode" align=middle width=153.5206827pt height=16.438356pt/></p>


---

## Interactive Feedback PID Loops
Interactivity depends on derivative constant <img src="svgs/c4dd4df1478960c5f0d78f517ad773e5.gif?invert_in_darkmode" align=middle width=20.804288999999986pt height=22.465723500000017pt/> as well as the dependence between proportional, integral and derivative modes. Parallel (non-interactive) controllers, are mostly unused. PID controllers are given by the alg.: 

<p align="center"><img src="svgs/acd9a664591bf03c3e7c0178c1f8c1df.gif?invert_in_darkmode" align=middle width=294.93900314999996pt height=41.004999749999996pt/></p> 
<br>

<p align="center"><img src="svgs/6bb343272fe5c4d583f87758cdd926b7.gif?invert_in_darkmode" align=middle width=225.0457011pt height=118.35736770000001pt/></p>

### **PIDF**: PID under a closed loop feedback control. Uses basic concept of setpoint, error, and target to generate clamped output, applied to system to reach target. 
- Error `e` multipled by proportional constant `K_p`, rendering output proportional to error
- Error is integrated over period `Δt`, then multipled by constant `K_i`. Renders the output on accumulation of past errors
- <img src="svgs/9290c8cfe86c39362512451d3a56f5a0.gif?invert_in_darkmode" align=middle width=34.72612439999999pt height=24.65753399999998pt/> of error `e` (calculated as `prevError - error`) normalized over update frequency (e.g. `Δdt = 10hz`) then multiplied by derivative constant `K_d`. Renders output on basis of error rate of change
- Finally, feed-forward determines relative "push force" to reach setpoint. Calculated by multiplying setpoint by constant `K_f`. Used less in PI/PID loops, more in motion-profiling velocity loops to determine initial output


## Integral Windup
Situation in PID loop where a large change in setpoint offests large rise (windup) in integral term `i`
- Rather than post-processing integral output, windup regulates output in real time

![](./media/img1.png)
**Figure 1.0** Comparison of anti-windup strategies and setpointing

#### Clamping
Also known as conditional integration, uses technique to clamp range of integral output from accumulating in wrong direction during output saturation. <br>
**process saturation**: situation in which PI controller becomes non-linear and unbounded
- Avoided through `(1) integral anti-windup (2) gain scheduling (3) setpoint weighting`
- *Integral anti-windup is best used to prevent oscillations in robot movements and consistency over autonomous routines*

> Trivial unwind calculations for small changes in error, using clamping
```cpp
#define unwind 0.001 // 0.001 -> 0.005
if((error > 0.0 && errTot < 0.0) || (error < 0.0 && errTot > 0.0) || fabs(error) < 0.001){
	  	if((fabs(error) - unwind) > -0.001) errTot = 0.0;
    }
if(fabs(unwind) < 0.001 && fabs(error) < 0.001) errTot = 0;
```

#### Back-Calculation
Using feedback `difference |u-v|` as input for integral. Back-calculating integral to constrain process output before feeding into integral. According to `Fig. 1.0`, back-calculation typically results in underaccumulation and drop-off

---

## Tuning
Tuning of constants and sensors is crucial to long-term accuracy and oscillation-reductions<br>
The general strategy for non-feedback controllers involves:
- Setting `I` and `D` to zero, `P` to a small (to be increased) value
- Increase `P` until oscillations are reduced
- Increase `D` until oscillations are nearly minimized
- If steady-state (`S-S`) error (setpoint never reached) observed, decrease `P` / increase `D` and add `I` 

For feedback controllers:
- In a velocity control loop, set P=I=D=0 initially, then increase feed-forward term until a desired velocity is reached
- Follow non-feedback steps for target, overshooting, undershooting, oscillations ...
- Increase D slowly to smooth out response (force under-dampening)
- Use integral gain for constant, noticable difference from setpoint
    * Often not used: can cause destabilization

> `Most systems, particularly subsystems of systems, do not need I or D depending on the movement and relationship between target and current state`

> ### Behaviorial effects of **increasing** K<sub>p</sub>, K<sub>i</sub>, K<sub>d</sub>, on general equation of interactive PID
<p align="center"><img src="svgs/9d619e79f11f2d061a3213f909ac606f.gif?invert_in_darkmode" align=middle width=335.8670469pt height=39.452455349999994pt/></p><br>
<span style="color:gray">*Effect on U(s) behavior*</span> 

| Response  | Rise Time | Overshoot  | Settling Time | S-S Error |
| :-------------: |:-------------:| :-----:| :-----:| :-----:|
| K<sub>p</sub> | decrease | increase | *NC | decrease
| K<sub>i</sub> | decrease | increase | increase | eliminate
| K<sub>d</sub> | *NC | decrease | decrease | *NC
**NC**: Negligable change

<center><img width="450" src="./media/tuning.gif"></center>

---

## Pure Pursuit
Pursuit comes in two different forms: adaptive and pure pursuit. "Pure" refers to pursuit directed purely on the basis of lookahead. However, "adaptive" pursuit modifies the look-ahead point over larger distances e.g. proportionally scaling lookahead with tracking error<br>
> Debately the best method of autonomous control (competitive scenarios) beacuse of overall reliability, accuracy, and abililty to alter paths on the fly
- Pure Pursuit formed from P controller using heading as setpoint and current heading as input. The goal point is derived from a fixed lookahead distance. 
   * `proportional gain` normalized by fixed lookahead distance `L`. *The curvature of movement comes from PI on heading rather than distance* : <img src="./media/eq1.gif?invert_in_darkmode"> determines the average curvature `C` on the path
   * A limiter is used to clamp (limit) curvature at high-speeds to prevent unwanted deviation from the path. This takes on the effect of cutting curve lines if implemented incorrectly. One method of limitation is regulating the angular velocity ω on sharp curves | often a threshold is placed on the entire path because of inability to track ω at high speeds
      - <img src="./media/eq2.gif?invert_in_darkmode">
   * Errors caused by too short or too long of `L` result in overall instability | solved with feedforward controllers (PIDFs)<br>

> **Warning**: Increasing lookahead distance on unsmooth curves will cause corner-cutting
- Smoother paths are preferred for the reason of more accurate prediction -> optimal path-finding prediction

<center><img width="375" src="./media/img5.png"></center> 

**Figure 1.1** Pure pursuit, no adaptivity
   
- Adaptive pure pursuit computes a running vector of the point on the path closest to the current vehicle position.
   * Using the `monotone arc length assumption`, efficiency can be increased by not having to search the entire length of the path for a lookahead, but rather using a local heading
   * Lookahead distance is adaptive / proportional to current tracking error

<center><img width="375" src="./media/img4.PNG"></center>

**Figure 1.2** Adaptive Lookahead, advanced off-path correction

### Path Following
1. Determine closest point
2. Determine lookahead point
3. Calculate (current) curvature, with target as lookahead point
4. Calculate & feed into controller: individual wheel velocities (vel. PID)



### Point Injection
The spacing of points along a path (predetermined) can be placed with minimal padding to increase the accuracy of velocity setpoints. Smoothing is best achieved with an increased quantity of points. Injection algorithms determine the spacing between points and pre-calculate the points along a pre-determined segment

```cpp
padding = // padding between points
v_gen_points = // generated points
for segment in path:
   vector = endpt - startpt
   points_in_range = ceil(magnitude(vector) / padding)
   vector = normalize(vector) * padding
   rep(i, 0, points_in_range):
      v_gen_points.append(startpt + vector*i)
```

### Smoothing
Smoothing can be achieved through several methods. Most common is through an optimization of points by returning a set of points that create a smooth trajectory. <br>
**The following algorithm uses gradient descent with a low tolerance:** <br>

<p align="center"><img src="svgs/44db4483891f4353404b2fffac602336.gif?invert_in_darkmode" align=middle width=144.5793822pt height=16.438356pt/></p> (1)
<center>Gradient using coordinates from un-smoothed trajectory</center> <br>

<p align="center"><img src="svgs/cff38cf230d636154e13fcf1f9fef39a.gif?invert_in_darkmode" align=middle width=218.145939pt height=16.438356pt/></p> (2)
<center>Gradient with respect to neighboring, smoothed trajectory coordinates</center>

<!-- <center><img src="svgs/cff38cf230d636154e13fcf1f9fef39a.gif?invert_in_darkmode" align=middle width=218.145939pt height=16.438356pt/></p> </center> (2) -->
<br>

<p align="center"><img src="svgs/2cba1ef8b323afd1e214c5dd574f834f.gif?invert_in_darkmode" align=middle width=333.81921704999996pt height=49.315569599999996pt/></p>

```python
""" path: set of path coords.
    weight_data: weight to update data (alpha)
    weight_smooth: weight to smooth coordinates (beta)
    tolerance: iteration delta """
newP = deepcopy(path) # smoothing each element in path
delta = tolerance  #0.00001

while delta >= tolerance:
    delta = 0.0
    for i in range(1, len(newP) - 1):
        for j in range(len(path[0])):

            x_i = path[i][j]
            y_i, y_p, y_n = newP[i][j], newP[i - 1][j], newP[i + 1][j]

            y_i_saved = y_i
            y_i += weight_data * (x_i-y_i) + weight_smooth * (y_n+y_p - (2 * y_i))
            newP[i][j] = y_i

            delta += abs(y_i - y_i_saved)
return newP
```

**Alternatives e.g. 5<sup>th</sup>-degree splines work for this approximation :**

<center><img width="250" src="./media/imgSpline5.PNG"></center>

### Closest Point
`Trivial` distance calculation, start at lowest index such that following closest points are sequential

### Lookahead Point
Point `P` separated by a `lookahead distance` from the current position. This point is determined by taking a circle with radius `lookout distance` and computing the intersection of the path and the circle `(line segment collision detection on circles)`. 

<p align="center"><img src="svgs/0c6cb1b5821721b71d1b044c7ed5c1e0.gif?invert_in_darkmode" align=middle width=103.04318805pt height=16.1187015pt/></p>

<br>

<p align="center"><img src="svgs/05c7c7860ce576ce1b2cf15cbc523d45.gif?invert_in_darkmode" align=middle width=374.67543629999994pt height=69.0417981pt/></p>

<br>

<center><img width="250" src="./media/img6.png"></center>

**Figure 1.3** Red circle defines the lookahead point `lookahead distance` from the center of movement

### 


<!-- ---  -->
<!-- ## Advanced Techniques in Pursuit  -->
<!-- clothoids
https://journals.sagepub.com/doi/full/10.5772/61391 -->
<!-- ... -->

---
## Motion Profiling
Motion profiling is similar to the path generation of pursuit algorithms, in that a series of points are given as input, and a smoothed, curved path is computed as output. However, profiling involves returning a path with parameters of acceleration and deceleration passed directly into motion functions. The steps to the this generation are as follows:<br> 
(1) Generating smoothed curve using techniques as defined in path pursuit <br>
(2) Calculting position through odometry functions <br>
(3) Generating acceleration along the curve <br>

### Curve and Path Generation
Smoothed curves are generated through the fucntional input to cubic bezier curves. Linear interpolation is a common method to determine a common pivot between two points. All adjacent points are interpolated until none remain <br>
(1) 1D linear interpolation <br>

<p align="center"><img src="svgs/151aa3393aa0743c29b35a137f9719bd.gif?invert_in_darkmode" align=middle width=245.81355645pt height=16.438356pt/></p> <br>

(2) 2D linear interpolation given two points P1, P2<br>

<p align="center"><img src="svgs/314b40797f433682dfda9e2215cfec79.gif?invert_in_darkmode" align=middle width=367.27787414999995pt height=17.8831554pt/></p>

<br>

<center><img width="300" src="./media/img7.PNG"></center>

**Figure 1.4** Guided cubic Bezier curve between two points

Bezier curves require a combination of interpolation at two end points, and approximation of the inner control points. For larger sets of points, a single curve can not accurately represent a smooth path because of derivative scaling. However, a solution involves splines through `quintic Hermite splines` which alot for multi-segmented curves. At joint points in the spline, a tanget rather than a control point is specified which reliable smoothes those points and maintains first derivative which cubic Beziers are unable to provide. 

### Velocity and Acceleration Profiles
Given a standard trapezoidal velocity profile, the points given non-standard acceleration are near the start- and end-points. Using two-step PID loops on acceleration, a jerk-proof profile can be achieved which minimizes the third-derivative of position and further smoothes motion. <br>
Trapezoidal profiles can be represented as simple kinematic equations:<br>

<p align="center"><img src="svgs/d631dd36cb0af6dffca8ee3b4a99e3f0.gif?invert_in_darkmode" align=middle width=161.60679975pt height=32.990165999999995pt/></p>
<p align="center"><img src="svgs/2e178cb8b27831b4b1ae09bff76a8630.gif?invert_in_darkmode" align=middle width=99.25597275pt height=16.438356pt/></p>

> Jerk is defined as transitional changes in acceleration, that produce vibrational forces and jolted acceleration. Jerk-reducing profiles are difficult to tune as their deceleration stages must follow the limitations of hardware without significantly increasing settling time or accuracy at slow speeds. Jerk profiles include several more stages than simple trapezoidal profiles: <br>
(1) linearly increasing accelearation to reach maximum acceleratoin (often guided by second-stage feedback controllers) <br>
(2) as system approaches max `V` acceleration must settle
(3) acceleration is constant at peak velocity until the inverse of (1) and (2) occur until final settle

S-curve / Jerk-reduced profiles are represented as: 

<p align="center"><img src="svgs/38bedef0628f7b8509ce41c60b9e1b22.gif?invert_in_darkmode" align=middle width=208.9472682pt height=32.990165999999995pt/></p>
<p align="center"><img src="svgs/7a56fa444da47714f3deec9687219e8f.gif?invert_in_darkmode" align=middle width=151.71062444999998pt height=32.990165999999995pt/></p>
<p align="center"><img src="svgs/e38e1bc1a250e8daaac073bf19b8a72a.gif?invert_in_darkmode" align=middle width=99.12963554999999pt height=16.438356pt/></p> 
<br>


<center><img width="500" src="./media/img8.PNG"></center>

**Figure 1.5** S-Curve Profile

### Following Generated Profiles
The final profiles sent to wheels is procaessed first through a series of offset conversions that returns a requirement for each point P in the form `(position, velocity, acceleration) at time t`. This point data is fed into the feedback controller and translated to scaled motor input. 

<!-- kalman filter? -->

---

# Final Notes
In a single system the previous implementations can be implemented in the following way: <br>
> (1) Pre-calculated paths and smoothing constants <br>
> (2) Path data is read, and curvature and velocity setpoints are computed <br>
> (3) Using odometry, the lookahead point is calculated and absolute positioning is maintained <br>
> (4) Curvature, closest point, lookahead, wheel velocities, and telemetry are all tracked in real-time <br>

PID, State-spaces, feedforward and motion profiling, and pursuit methods all define a broader spectrum of control theory that is applicable to more than purely robotic applications (e.g. self-driving cars, AI-based intelligence). Modern multi-in-multi-out control systems are founded on these concepts and continue to be developed. 

