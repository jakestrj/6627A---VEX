Motion Profiling, Tracking via Odometry Theory on Fixed-Differential Systems
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
Movement from: $\left \langle \bar{x}, \bar{y}, \bar{\theta} \right \rangle to \left \langle \bar{x'}, \bar{y'}, \bar{\theta'} \right \rangle$ <br>
Odometry telemetry: $u = \left\langle x, y, \theta \right\rangle$ (absolute at time $t$)

### Modeling Motion
$$\Delta s = \frac{\Delta s_r + \Delta s_l}{2} $$ 

<br>

$$\begin{cases}
   \Delta s_r = \text{right wheel base delta} \\
   \Delta s_l = \text{left wheel base delta}
\end{cases} $$

<br>

$$\Delta \theta = \frac{(\Delta s_r - \Delta s_l)}{2L} $$ 

<br>

$$\begin{cases}
   \Delta s_r = \text{right wheel base delta} \\
   \Delta s_l = \text{left wheel base delta} \\
   2L = \text{wheel base difference}
\end{cases} $$

<img src="./media/img2.png" width="300"> <img src="./media/img3.png" width="384">

$$\Delta x = \Delta scos\left( \theta + \Delta\theta/2 \right)$$ <br>

$$\text{(Change in Abs. X)}$$ <br>

<br>

$$\Delta y = \Delta ssin\left( \theta + \Delta\theta/2 \right)$$ <br>

$$\text{(Change in Abs. X)}$$

<br>

$$p' = f(x,y,\theta,\Delta s_r, \Delta s_l) = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} + \begin{bmatrix} \Delta s*cos(\theta + .5*\Delta\theta \\ \Delta s*sin(\theta + .5*\Delta\theta \\ \Delta\theta \end{bmatrix}$$
$$(Position-Change)$$


---

## Interactive Feedback PID Loops
Interactivity depends on derivative constant $K_d$ as well as the dependence between proportional, integral and derivative modes. Parallel (non-interactive) controllers, are mostly unused. PID controllers are given by the alg.: 

$$U(t) = K\left ( e(t) + (\frac{1}{T_i} \int_{0}^{t}e(\alpha)) + K_d(s) \right )$$ 
<br>

$$\begin{cases}
   U(t)=\text{controller output} \\
   E = \text{target} - \text{input for time } t \\
   K_p = K\\
   K_i = \frac{K}{T_i} \\
   K_d = KT_d
\end{cases} $$

### **PIDF**: PID under a closed loop feedback control. Uses basic concept of setpoint, error, and target to generate clamped output, applied to system to reach target. 
- Error `e` multipled by proportional constant `K_p`, rendering output proportional to error
- Error is integrated over period `Δt`, then multipled by constant `K_i`. Renders the output on accumulation of past errors
- $d/dx$ of error `e` (calculated as `prevError - error`) normalized over update frequency (e.g. `Δdt = 10hz`) then multiplied by derivative constant `K_d`. Renders output on basis of error rate of change
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
$$U(s) = G_{pid}(s)E(s) = \left ( K_p + K_i (\frac{1}{s}) + K_d(s) \right )$$<br>
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

<center><img width="375" src="./media/img4.png"></center>

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

$$y_i \rightarrow y_i + \alpha(x_i-y_i)$$ (1)
<center>Gradient using coordinates from un-smoothed trajectory</center> <br>

$$y_i \rightarrow y_i + \beta(y_{i+1} + y_{i-1} - 2y_i)$$ (2)
<center>Gradient with respect to neighboring, smoothed trajectory coordinates</center>

<!-- <center><img src="svgs/cff38cf230d636154e13fcf1f9fef39a.svg?invert_in_darkmode" align=middle width=218.145939pt height=16.438356pt/></p> </center> (2) -->
<br>

$$\begin{cases}
   \alpha = \text{smoothing optimization parameter} \\
   \beta = \text{weight (emphasis) smoothing parameter}
\end{cases} $$

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

<center><img width="250" src="./media/imgSpline5.png"></center>

### Closest Point
`Trivial` distance calculation, start at lowest index such that following closest points are sequential

### Lookahead Point
Point `P` separated by a `lookahead distance` from the current position. This point is determined by taking a circle with radius `lookout distance` and computing the intersection of the path and the circle `(line segment collision detection on circles)`. 

$$I_p = E + \tau * d$$

<br>

$$\begin{cases}
   E = \text{starting point} \\
   \tau = \text{intersection along ray of path} \\
   d = \text{vector representing ray of path (line segment)}
\end{cases} $$

<br>

<center><img width="250" src="./media/img6.png"></center>

**Figure 1.3** Red circle defines the lookahead point `lookahead distance` from the center of movement

### 


--- 
## Advanced Techniques in Pursuit 
<!-- clothoids
https://journals.sagepub.com/doi/full/10.5772/61391 -->
...

---
## Motion Profiling
<!-- Motion profiling is similar to path pursuit, however  -->

# Final Notes
In a single system the previous implementations can be implemented in the following way: <br>
> (1) Pre-calculated paths and smoothing constants <br>
> (2) Path data is read, and curvature and velocity setpoints are computed <br>
> (3) Using odometry, the lookahead point is calculated and absolute positioning is maintained <br>
> (4) Curvature, closest point, lookahead, wheel velocities, and telemetry are all tracked in real-time <br>

PID, State-spaces, feedforward and motion profiling, and pursuit methods all define a broader spectrum of control theory that is applicable to more than purely robotic applications (e.g. self-driving cars, AI-based intelligence). Modern multi-in-multi-out control systems are founded on these concepts and continue to be developed. 

