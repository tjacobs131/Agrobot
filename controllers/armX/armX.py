from controller import Robot, Keyboard, Supervisor

# Initialization
robot = Robot()
keyboard = Keyboard()
TIMESTEP = 64

# Linear Motor
lm = robot.getDevice("lmX")
linear_speed = 0.0 # The position of the arm relative to the SliderJoint

while robot.step(TIMESTEP) != -1:
    # if linear_speed > 1:
        # linear_speed += 0.05
    # elif linear_speed :
    linear_speed -= 0.05
        
    lm.setPosition(linear_speed)
    
pass