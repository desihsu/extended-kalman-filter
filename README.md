# Extended Kalman Filter

Uses an [extended kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) to estimate the state of a moving object of interest  
with noisy lidar and radar measurements. A Jacobian matrix is used for nonlinear  
state estimation for radar measurements.

## Build Instructions

1. Clone this repo
2. Download [simulator](https://github.com/udacity/self-driving-car-sim/releases/)
3. Install [uWebSocketIO](https://github.com/uNetworking/uWebSockets): ```~$ sh install-ubuntu.sh```
4. ```~$ mkdir build && cd build```
5. ```~$ cmake .. && make``` 

## Usage

1. Run the simulator (EKF)
2. Connect: ```~$ ./ExtendedKF```
3. Press start

