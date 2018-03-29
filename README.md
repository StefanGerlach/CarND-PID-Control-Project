# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program



## Reflection of this project
In this section I want to reflect how I set the PID-Controller parameters.

First of all I implemented the pipeline of the PID-Controller, to compute the steering angle of the simulator vehicle based on the current cross track error (CTE). The PID-Controller consists of 3 parameters tp, td, ti, that define the impact of the propotional, differential and integral part of the CTE on the new computed steering value.

To manually tune the parameters, I played around with different values for the 3 parameters. The parameter for the proportional part leads to 'harder' steering when the vehicle is 'off' the desired track. A higher value leads to the effect, that the vehicle is steering into the direction of the desired track, but also to an oscillation and overshoot. 

To smoothen the steering, the differential part can be used. Additionally, to compensate an offset in the steering of the vehicle, the integral part of the PID-Controller is used.

I left the speed of the vehicle at 0.3 (default). With manual tuning I solved the track with the paramter set to tp=0.4, td=0.0, ti=6.5.


## Optimize Parameters
To optimize the parameters I imlemented a pipeline for automatic optimization. I defined a maximum-error of err_max=2.0, and if this error is exceeded on the track, the simulator is restarted for the next tuning-iteration. To reset the simulator, I used the "reset" command.

Additionally, I defined a maximum numbers of 'onMessage' events, max_steps=7000. This is approximately the complete course. If the complete track is finished, the simulator is restarted for the next tuning-iteration, too.


## Loss / Score - Function to optimize
I used the following metric to define my gain/score-function:

error_score = 1.0 - (current_error / max_error)

distance_score = current_distance / max_steps

gain = ((10 * distance_score) + (1.0 * error_score))  / 11.0 

My goal was to maximise the gain. As described in the formula, I weighted the distance score with a factor of 10, because the car should definitely complete the track. 


## Optimize Parameters with Grid Search
To optimize the parameters, I wrote the Optimizer class. This class is used to do a grid search in a defined parameter space. I used the minimum values of tp_min=0.0, td_min=0.0, ti_min=0.0 and maximum values of tp_max=1.0, td_max=0.1, ti_max=7.0 with using the steps of tp_step=0.125, td_step=0.02, ti_step=0.75. 

This leads to 6561 parameter combinations.

## Optimize Parameters with Coordinate Ascent
When the best parameters are found after the grid search steps, an algorithm like the 'Twiddle' or 'Coordinate Ascent' is used to fine-tune the parameters. The algorithm is described in the /src/optimizer.cpp OptimizeParameter() function. 

Generally spoken, it checks after each tuning-iteration if the current score is greater than the best known score. If yes, the tuning direction of the current 'to-tune-parameter' is kept and the value is adjusted again. If the current score does not reach the best score, the current 'to-tune-parameter' is reset to the value before, and the step_size to adjust this paramter is reduced. The step_size is used again to tune this parameter. If the step_size reaches a lower limit, and no increasement of score was achieved, the 'to-tune-parameter' is changed.


## Results of Optimization
Unfortunately the maximum resets of the simulator were about 1014 times, then the program freezed and I could not find out why. So the grid search was incomplete and landed at about tp=0.5, td=0.0, ti=0.0. This was my starting point for the fine tuning. This also could not complete the (very time consuming) process, so the result was about tp=0.6, td=0.0, ti=0.0.

Not the desired optimization goal. But I left the values at tp=0.3, td=0.00001, ti=7.0 to complete the track quite smoothly with 0.3 as the speed parameter.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

