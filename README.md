# quadrotor-control-and-trajectory-generation
## 1D Control of Quadrotor
We implemented a PD controller for height control of the quadrotor. We tuned the proportional gain(kP) and derivative gain(kD) for two different cases. 



### Hover Control
The first case was of hover control where the quadrotor was simply stabilized at the height of 0m. 
![](https://i.imgur.com/sjb3Hax.gif)


### Step Response
The second case was of step response where the quadrotor was given an input height of 1m.
The rise time was kept as less than 1 second and the maximum overshoot was capped at 5%. 
![](https://i.imgur.com/hUNL5Cy.gif)



## 2D Control of Quadrotor
After controlling quadrotor in 1 Dimension, our next objective is to build a PD controller to control the motion of quadrotor in Y-Z (2D) plane. Below are results ,we moved the quadrotor from some y postion to another, keeping height (z) constant.

### Hover Controller
The Quadcopter was moved to a desired position in y direction
![](https://i.imgur.com/RBx20Oo.gif)

### Trajectory Controller

The Quadcoptor followed the desired trajectory.
![](https://i.imgur.com/alDXe4I.gif)

## 3D Control of Quadrotor

Since now we have applied 1D and 2D to our Quadcopter, Now we will be extending it to 3D control and then to trajectory generation. We will firstly make the drone to follow Line and Helix trajectory, whose corresponding gifs are shown below. 

**The position and attitude control loops can be given as:**

![](https://i.imgur.com/4aPewDa.png)


### Line Trajectory

![](https://i.imgur.com/IXZikkK.gif)

#### **Quad Velocity**
![](https://i.imgur.com/giXTM5l.jpg)

### **Quad Position**
![](https://i.imgur.com/5B75MVU.jpg)


### Helix Trajectory

![](https://i.imgur.com/U2YcQ1g.gif)

#### **Quad Velocity**
<!-- ![](https://i.imgur.com/5pvjgz5.png =500x300) -->
<img src="https://i.imgur.com/5pvjgz5.png" width='560' height='420'/>




#### **Quad Position**
![](https://i.imgur.com/nsXInF3.jpg)


### Trajectory Generation

We generated a minimum snap trajectory using a $7^{th}$ order polynomial. In Trajectory generation we are given a cerain number of waypoints, and the drone has to follow the path from one way point to other. In this way if we increase the number of waypoints bascially we can improve drone's trajectory.

![](https://i.imgur.com/agvmTQC.gif)

#### **Quad Velocity**
![](https://i.imgur.com/m5QtAeB.jpg)

#### **Quad Position**
![](https://i.imgur.com/426y7HL.jpg)
