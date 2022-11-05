#!/usr/bin/env bash

rosservice call /oarbot_silver/imu_gravity_compensation/calibrate_bias;
rosservice call /oarbot_blue/imu_gravity_compensation/calibrate_bias;