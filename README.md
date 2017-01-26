# SLAM-HW1-Public

## Introduction

This is the readme file of the first homework (HW) of the SLAM course of the UVIC robotics master.

The objective of this HW is to build a ROS node that integrates motion data into a odometry trajectory, and also into motion factors to be used in a factor graph.

## Rationale

We basically integrate motion to produce two kinds of data: 
  - the motion since the beginning of time, what we call 'global pose', and 
  - the motion since the last keyframe, what we call 'pose increment'. 
  
This is a sketch of the situation:
```
       D_01            D_12            D_23           D   d
       C_01            C_12            C_23          C_D  C_d
KF_0           KF_1            KF_2            KF_3       X   
 * - - - - - - - * - - - - - - - * - - - - - - - * - - - ->
t_0             t_1             t_2             t_3       t
```

where:
  - we are currently at time `t`.
  - the last motion data produces the current motion increment `d=[dx,dy,dth]`. This has an associated covariance `C_d`.
  - we have already 4 keyframes, `KF_0` to `KF_3`. These are of the type `KF=[x,y,th]`.
  - we have computed 3 pose increments, `D_01` to `D_23`. These are of the type `D=[D_x,D_y,D_th]`. We also computed their covariances `C_*`.
  - we are on our way to compute the next pose increment, now `D` and `C_D`, which is basically a composition of the increment integrated so far, `D`, and the current increment `d`,
  ```
  D <-- D (+) d
  ``` 
  
   where `(+)` is a frame composition operation, which in 2D is fairly easy to implement (see PDF, eqs. 2.12 and 2.26).  
   We integrate the covariance `C_D` through the Jacobians (see PDF, section 2.3.2, and read further below this document).  
   This `D` will lead in the near future to the next keyframe, `KF_4`, thus becoming `D_34`. 
  - we have also integrated a global pose `X=[x,y,th]`, that is, the integration of all the increments. 
  Technically, we have `X = D_01 (+) D_12 (+) D_23 (+) D`, but we actually perform an incremental integration: at each time step, simply do 
  ```
  X <-- X (+) d
  ```

  and publish the current global state, `X`.

  - At the arrival of the new keyframe `KF_4`, we need to publish `D_34 = D`, together with its covariances matrix `C_34 = C_D`. Then, we can reset these integrals `D` and `C_D` so that we can proceed with the next pose increment.

## ROS

Ubuntu 16.04.1 LTS and ROS Kinetic are the distro and framework version of choice for this course. It is understood that students at this stage should have completed an installation of Kinetic along with some of the rudimentary ROS tutorials featured on [ros.org](ros.org). Regardless, here are the relevant links to get you started:

 - [Ubuntu 16.04.1 LTS](http://releases.ubuntu.com/16.04/ubuntu-16.04.1-desktop-amd64.iso)
 - [ROS Kinetic Installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)
 - [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
  - [Installing and Configuring ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Next, follow these instructions to acquire the homework skeleton code and configure your system, in the aforementioned way.
Firstly, open a terminal, go to your home directory and create a catkin workspace for this and any future homework.
```
  $ cd ~
  $ mkdir -p slam_ws/src
```
Next, we have to initialize our workspace.
```
  $ cd slam_ws/src
  $ catkin_init_workspace
```
And, just to check, let's try to compile the empty workspace.
```
  $ cd ..
  $ catkin_make
```
You should see a build screen, and if there isn't any red, you're in the clear and can proceed.
Now, we gather our skeleton code from github. Make sure you have git installed also.
```
  $ sudo apt-get install git
  $ cd src
  $ git clone git@github.com:davidswords/SLAM-HW1-Public.git
```
With our skeleton code in the source directory, we can attempt to build it with catkin. 
```
  $ cd ..
  $ catkin_make
```
You should see a building screen, this time not so blank, but detailing the packages and nodes found. If no red is to be seen, then we are in the clear to proceed.

To run the skeleton code, we would like you to use the roslaunch files provided. There aren't any necessary modifications needed for this homework to any file besides "odometry/odometry.cpp", and your homework should be run with the following command:
```
  $ roslaunch odometry main.launch
```
Once the above is run, it will execute each node in your odometry package, as well as bring up a configured rviz.
As mentioned previously, the file you will be working is odometry.cpp, this can be reached easily with the following:
```
  $ roscd odometry/src
```
A teleop node has been wrapped in a roslaunch for your convienience. This will allow you to send velocity commands to your odometry node. You can run, in a separate terminal to your main.launch, with the following:
```
  $ sudo apt-get install ros-kinetic-teleop-twist-keyboard
  $ roslaunch odometry teleop.launch
```
Once main.launch is running, and you're operating your odometry node, you will want to see the contents of your rostopics, you can view which topics are available with the following command:
```
  $ rostopic list
```
After running this, you should see the following available topics:
```
/cmd_vel                (Velocity commands coming from the teleoperation node)
/odometry/C_D           (The Delta covariance matrix update)
/odometry/Delta_pose    (The Delta pose update, factor integration)
/odometry/delta_pose    (The delta pose update, velocity integration)
/odometry/global_pose   (Global pose update)
/tf/global_pose         (Global pose integrated into a tf model for visualization in rviz)
```
If all of this works without a hitch, then you are good to go on developing your odometry module (odometry/odometry.cpp).

## Homework

Once you have your workplace set up and running, you need to complete the code in `odometry.cpp` to achieve the full functionality explained above:
  1. Initial setup, through the constructor of the class `Odometry`.
  1. Acquisition of twist data through the `callback` (read on for the definition of twist)
  1. Integration of the twist data, through the `run` function (read on for guidelines)
    - create the current motion increment, `d`
    - integrate the pose increment `D` -> factor integration
    - integrate the global pose `X`    -> current robot pose
  1. Publish current global pose `X` at each time instant (this piece of data may be required by the robot planner and controller)
  1. Publish pose increment `D` and `C_D` (this piece of data is required to construct the factor linking the two keyframes). Mind that you need to publish:
    - The Pose Increment itself, `D`
    - the pose increment's Covariances matrix, `C_D`
    - You may want to reset the pose increments every now and then (for example, every N time steps). This reset involves:
      - The Pose Increment itself, `D`
      - the pose increment's Covariances matrix, `C_D`
  
Finally, visualize the integrated poses using rviz
  - The global pose `X`
  - the pose increment `D`
and see how the increments integrate just pieces of trajectory. These will be the motion factors in the graph-SLAM to be built over this course.

The following sections of this README file provide substantial quidelines to complete this HW. 
Please refer also to the PDF documentation of the course, especially Chapter 2, Sections 2.1, 2.2, 2.3.4 and 2.3.2 (we recommend having a thorough read of these sections beforehand, and in this order).

<i><b>NOTE</b>: Variable names in this README file do not necessarily match those in the code file or the PDF file. You can however follow these guidelines:
  - Variables without prefix are global: `x, t, th, vx, vy, w`.
  - Variables with a `d` prefix are individual increments: `d_t, d_x, d_y, d_th`.
  - Variables with a `D` prefix are integrated increments: `D_x, D_y, D_th`
  - Variables with a `C` prefix are covariance matrices.
  - Variables with a `J` prefix are Jacobian matrices.</i>

### Initial setup (can be done at construction time)

Global odometry pose
```
    x = y = th = 0;
```

Factor pose increment
```
    D_x = D_y = D_th = 0;
```

Factor pose increment's covariance
```
          | 0 0 0 |
   C_D  = | 0 0 0 |
          | 0 0 0 |
```
Twist data covariance (constant), using user-provided uncertainty parameters `sigma_*`
```
          | sigma_vx^2  0           0           |
   C_v  = | 0           sigma_vy^2  0           |
          | 0           0           sigma_vth^2 |
```

### Acquisition of the twist `(v,w)`

The motion data comes in the form of a twist, that is, linear velocity `v=[vx,vy]` and angular velocity `w`.

The twist is acquired by a `callback()` function by reading a message of the type `geometry_msgs::Twist`.

This ROS message is designed for 3D motion, but we use it in 2D. 
Since we are in 2D, we need to extract only the following data:
  - `vx`: read from  `linear.x`
  - `vy`: read from  `linear.y`
  - `w` : read from  `angular.z`
  
Usually, we will have `vy=0`, but we'll accept a non-null `vy` for greater genericity.

### Integration of the twist `(v,w)`

The integration is performed at regular time intervals `d_t`, in the periodic process `run`, using the last available twist `v`, `w`, and a constant `d_t`. 

The integration process is implemented through 3 separate steps:
  1. Time integration of twist data `v,w` over the period `d_t` onto the current pose increment `d`,        
  1. Integration of the current pose increment `d` onto a the pose increment `D`,    
  1. Integration of the current pose increment `d` onto the global odometry pose `X`,    

These three steps are detailed below.

#### Time Integration of the twist tata into the current pose increment `d = [d_x,d_y,d_th]`
See the PDF, section 2.3.4.

Compute the current time step
```
    d_t = t_now - t_previous
```
where `t_now` and `t_previous` are here just indications for the current time and the previous time. Find your way through the ROS timestamp utilities, or use a fixed `d_t` if you can (only if you use the `run()` approach).

Time-integrate the twist onto a pose increment `[v,w] , d_t --> [d_x,d_y,d_th]`
```
    d_x    = vx * d_t;
    d_y    = vy * d_t;
    d_th   = w  * d_t;
```
Compute the Jacobian of the above
```
             | 1 0 0 |
   J_dvdt  = | 0 1 0 |
             | 0 0 1 |
```
Compute the covariance of the current pose increment `d`
```
   C_d      = J_d_vdt * C_v * J_d_vdt^T * dt
```

#### Motion factor Integration `D = [D_x,D_y,D_th]`
See the PDF, section 2.3.2.

Integrate the factor's pose increment `[D_x,D_y,D_th] , [d_x,d_y,d_th] --> [D_x,D_y,D_th]`
```
   D_x 	   <- D_x  + d_x * cos ( D_th ) - d_y * sin ( D_th )
   D_y 	   <- D_y  + d_x * sin ( D_th ) + d_y * cos ( D_th )
   D_th    <- D_th + d_th
```
Jacobians of the equations above: with respect to `D`,
```
             | 1    0   -d_x * sin ( D_th ) - d_y * cos ( D_th ) |
   J_D_D   = | 0    1    dx * cos ( D_th ) - d_y * sin ( D_th )  |
             | 0    0    1                                       |
```
and with respect to `d`,
```
             | cos ( D_th )  -sin ( D_th )  0  |
   J_D_d   = | sin ( D_th )   cos ( D_th )  0  |
             | 0              0             1  |
```
Integrate the covariance of the factor's pose increment `D`
```
   C_D     = J_D_D * C_D * J_D_D^T + J_D_d * C_d * J_D_d^T
```

#### Global Pose Integration `X = [x,y,th]`
See the PDF, section 2.3.2.

Integrate global pose `[x,y,th] , [dx,dy,dth] --> [x,y,th]`
```
   x       <- x + d_x * cos ( th ) - d_y * sin ( th )
   y       <- y + d_x * sin ( th ) + d_y * cos ( th )
   th      <- th + d_th
```
Publish global pose
```
    >> x, y, th
```

### Resetting the factor's pose increment

When someone creates a keyframe, the odometry process needs to provide the factor integrated so far, and reset it to zero before starting the integration of the next motion factor.

In this HW we do not have Keyframes created yet, and therefore no real conditions for resetting the motion factor exist so far. So, we just simulate a reset every now and then (we recommend you do a reset every N integration steps). 

The reset involves the pose increment `D` and its covariance `C_D`,

```
    D_x = D_y = D_th = 0;
```
```
          | 0 0 0 |
   C_D  = | 0 0 0 |
          | 0 0 0 |
```

## Related documentation

PDF of the course:

http://www.iri.upc.edu/people/jsola//JoanSola/objectes/toolbox/graphSLAM.pdf
