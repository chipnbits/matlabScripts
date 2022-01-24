## MECH 223 Cassani Launch Simulator
Project version 5

### Lander Launch Trajectory Modeling



The landers can be launched from the cursor position or in an automated fashion from the orbiter.  For launches orgininating from the cursor on the table surface, use the top-left menu:


    
| Text Field | Description  |
| :--- | :--- |
| x Position | lander launch cursor x position on the playing surface |
| y Position | lander launch cursor y position on the playing surface |
| Velocity | Sets the launch velocity of the lander |
| Angle (Degrees) | Sets the intial angle of launch as measured in degrees from the x-axis |


- **Launch Lander**:
    Initiate a launch from the chosen intial parameters in the text fields.  The simulation will calculate the 3d trajectory of the lander and then plot it in the relative 2d and 3d viewports.  The telemetry information for all current and previously launched landers is held in the main workspace structure variable "landerData".  The output data records the following:    
    
    - x0 - initial x position at launch
    - y0 - initial x position at launch
    - vx0 - intial x-direction velocity at launch
    - vy0 - intial x-direction velocity at launch
    - t0 - time of intial launch, this value can be greater than 0 for orbiter initiated launches.
    - status - the final status of the lander when trajectory path calculations stopped.
        - stopped - the lander stopped motion due to reaching a 0 velocity
        - offtable - the lander left the competition space by exiting the outer perimeter
        - landed - the lander landed on titan successfully
        - active - this should represent a potential error, active means the lander was still moving in the competition space when pathing stopped        
        
    - funnelentry - contains the entry data for a lander that entered into the gravitational field of Titan:
        - theta0 - initial angle from center of titan, measured from postive x axis
        - r0 - initial radius of entry, should be the raius of titan for landers entering from the outside
        - dtheta0 - dtheta/dt relative to center of titan at entry
        - dr0 - the initial radial velocity at time of entry
        - lang0 - the initial angular moment at the point of entry.  In a frictionless environment, the angular momentum of the lander is completely conserved 
 
    - Telemetry contains all of the time and path information for the complete trajectory:
        - 1 - time since launch of either lander or orbiter
        - 2 - X velocity
        - 3 - Y velocity
        - 4 - Z velocity - (needs fixing)
        - 5 - X position
        - 6 - Y position
        - 7 - Z position

**Playback**:
    Animate the launch sequence and behavior using a rendering that follows the actual time positions.  The velocity of motion on the playing field is metered to match the actual telemetry.  Playback will play the most recent launch sequence activated, either lander or orbiter+lander.  Use this button to get a visual representation of the telemetry and results for both.
    

### Orbiter Launch Tracking and Modeling
The lower half of the user input interface is dedicated to orbiter launch functions.  An orbiter launch can have an activated trigger distance which is where it will autonomously deploy a lander launch aimed from the moving platform.  The sequence accounts for relative motion, lander launch velocities are entered as relative to the orbiter speed, static or in motion.

| Text Field | Description  |
| :--- | :--- |
| Trigger Distance | The distance at which the lander launch is triggered |
| Lander L Vel | The magnitude of the lander launch velocity |
| Launch Angle | Sets the launch angle for the lander, as measured from the x-axis of the orbiter |
| Friction | The coefficient of rolling friction to be used|
| Orbiter L Vel | The initial exit velocity for the orbiter launch |

- **Launch Orbiter**:
    An orbiter launch can have an activated trigger distance which is where it will autonomously deploy a lander launch aimed from the moving platform.  The sequence accounts for relative motion, lander launch velocities are entered as relative to the orbiter speed, static or in motion.  The orbiter trajectory is shown with crosshairs indicating set pathing time increments.  Default setting is .1s increments, the velocity and decellerations can be visualized in this way.  The launch sequence ends when the orbiter paths out of bounds or comes to a stop.  Orbiter dynamics have not been configured for entering titan!  Output data is recorded into the main workspace structure variable "orbiterData".  The output includes the following:
    - Path:
        - 1 - Time since launch
        - 2 - Vx
        - 3 - Vy
        - 4 - X Position
        - 5 - Y Position
        

### Interface Controls

- Lines On/Off controls the display of the regulated laser sites from the competition surface
- Reset Sim will remove all the visual trjectories and elements from the simulation surface.  Note that orbiter and launch data is still saved to the workspace until the program is closed.
    
        
    

