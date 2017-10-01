# Extended Kalman Filter Project 
---
This is my Project 1 in term 2 of the Ucatity Self-Driving Car Engineer Nanodegree Program, which aims to utilize an extended kalman filter to estimate the state of a moving object of interest with fused noisy lidar and radar measurements using C++. In this project I chose Ubuntu 14.04 LTS for implementation. 


[//]: # (Image References)

[image1]: ./results/success_dataset1.png 
[image2]: ./results/dataset1.png
[image3]: ./results/success_dataset2.png
[image4]: ./results/dataset2.png
[image5]: ./results/simulator.png


## Content of my project
---
This project origins from the [Udacity Starter Code repository](https://https://github.com/udacity/CarND-Extended-Kalman-Filter-Project),which includes [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) installation files __install-ubuntu.sh__ and several useful files as follows:

- **src folder**: with the project code.
  - **main.cpp** - reads data, calls functions to run the extendted Kalman filter and to calculate RMSE, and keep communication between simulator and uWebSocketIO.
  - **FusionEKF.cpp** and **FusionEKF.h** - initializes the filter, calls the predict function, and calls the update functions.
  - **kalman_filter.cpp** and **kalman_filter.h** - defines the predict function, the update function for lidar, and the updateEKF function for radar.
  - **tools.cpp** and **tools.h** - a function to calculate RMSE and the Jacobian matrix.
  - **measurement_package.h** - defines the data structure to process.
  - **json.hpp** - gives some communication protocols between simulator and uWebSocketIO.
- **data** folder: with input data file provided by Udacity.
- **Docs** folder: with files formats data structure.
- **ide_profiles** folders: with files sets up the development environment.
- **results** folder: with output screenshot.
- **README.md**: i.e. this project report you are seeing.

Note that the only files I need to modify are **FusionEKF.cpp**, **kalman_filter.cpp**, and **tools.cpp** for basic requirements in [project rubics](https://review.udacity.com/#!/rubrics/748/view). And the main protcol that main.cpp uses is the web socket server **uWebSocketIO** acting as a host, connecting the C++ programs to the Unity simulator, where the input values is provided by the simulator to the c++ program, and output values provided by the c++ program to the simulator.

The flow of FusionEKF is Prediction -> LIDAR update -> RADAR update, so I can narrow down the source of error when run into some problems.


## Simulation results
---
The laser measurements are denoted as red circles. The radar measurements are denoted as blue circles with an arrow pointing in the direction of the observed angle. And my estimation markers are denoted as green triangles as follows.

### Dataset 1

![alt text][image1]
![alt text][image2]

Evaluation Metrics - RMSE: [x,y,vx,vy] = [0.0974,0.0855,0.4517,0.4404]

The threshold in program rubics: RMSE <= [.11, .11, 0.52, 0.52] 

### Dataset 2

![alt text][image3]
![alt text][image4]

Evaluation Metrics - RMSE: [x,y,vx,vy] = [0.0726,0.0965,0.4219,0.4937]

The threshold in program rubics: RMSE <= [.11, .11, 0.52, 0.52]. Here use the same threshold as dataset 1.

---

## How to run this project
---

1. Clone the starter code repo [Udacity Starter Code](https://https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF `
5. Open the Unity Simulator 'term2_sim_x86_86', select the 'Project 1/2: EKF and UKF'. Terminal shows connection success. Choose dataset 1 or dataset 2. Then click start button.
![alt text][image5]


## Code Style
---
Here I stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html). And I use the ubuntu file editor gedit with the similar user interface with emacs. 


## Project Instructions and Rubric
---

Note: regardless of the changes you make, your project must be buildable using cmake and make! Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. See [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47) for instructions and the project rubric.