# FT calibration scripts
```
python ft_calibration.py
```
Reads csv file with joint values of kinova arm for the calibration orientations of the FT sensor. Jogs the robot to those poses one by one. At each point, Subscribes to the IMU messages and Wrench msgs reads them 100 times, takes the average (Note that from IMU, only the accelerations are needed, however they need to be on the same frame with wrench, so make sure they are published in the same frame, eg use a IMU transformer). After the averaging data is collected for that point with a matrix. After the all data is collected, a least squares estimate is used to estimate the sensor offsets (Fx,Fy,Fz,Tx,Ty,Tz) end effector mass(m) and center of mass coordintes in wrench frame (cx, cy, cz) is calculated and printed to the terminal as, for example: 

```
calib_residuals: [361.85667464]
mass: 1.2271697966535375
COM_pos: [ 0.0407208  -0.00243385  0.05250709]
F_bias: [-1.56365157 -3.58940157 -1.63655572]
T_bias: [ 0.06126408 -0.03329708 -0.1085475 ]
Calibration process ended successfully. You need to save these values to your ft calib data / gravity compensation parameter file..
```

For OARbots these parameters are needed to be saved to yaml files named `oarbot_blue_ft_calib_data.yaml` and `oarbot_silver_ft_calib_data.yaml` . This yaml file later can be used with a gravity compansation node to remove the static errors on the FT sensor. Oarbot launchers handles such compensation already but at every launch one also needs to call the bias calibration service with:
```
rosservice call /oarbot_blue/imu_gravity_compensation/calibrate_bias
rosservice call /oarbot_silver/imu_gravity_compensation/calibrate_bias
```

### Unknowns: 
- Bias (6x1), Gripper mass(scalar), CoM (3x1)
### Method:
- Create a csv file of robot poses at different orientations
- Jog robot to each pose one by one
- Use IMU data from Kinect for gravity vector
- Transform IMU to wrench frame
- Collect FT and IMU data at each pose.
- Create measurement matrix and vector based on Newton-Euler (static)
- After all poses visited apply Least Squares Estimation (LSE) for the unknowns

#### Newton-Euler (static)
$$
\begin{aligned}
&F_m^{(3 \times 1)}=-m g^{(3 \times 1)}+F_{\text {bias }}^{(3 \times 1)} \\
&T_m^{(3 \times 1)}=g^{\times} m c^{(3 \times 1)}+T_{\text {bias }}^{3 \times 1}
\end{aligned}
$$

#### Newton-Euler in matrix form to structure for LSE 
$$
Z = Hx
$$

where; 

$$
Z=\left[\begin{array}{l}
F_m^{(3 \times 1)} \\
T_m^{(3 \times 1)}
\end{array}\right]^{6 \times 1},
$$

$$
H=\left[\begin{array}{cccccccccc}
-g_x & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
-g_y & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
-g_z & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & -g_z & g_y & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & g_z & 0 & -g_x & 0 & 0 & 0 & 0 & 1 & 0 \\
0 & -g_y & g_x & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{array}\right]^{6 \times 10}
$$ 

and,

$$
x=\left[\begin{array}{l}
m \\
m c_x \\
m c_y \\
m c_z \\
F_{\text { bias }}^{(3 \times 1)} \\
T_{\text {bias }}^{(3 \times 1)}
\end{array}\right]^{(10\times 1)}
$$
