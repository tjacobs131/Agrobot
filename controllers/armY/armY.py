from controller import Robot, Keyboard

robot = Robot()
keyboard = Keyboard()
TIMESTEP = 64

lm = robot.getDevice("lmY")
linear_speed = 0.0

while robot.step(TIMESTEP) != -1:
    if linear_speed > -2.45:
        linear_speed -= 0.05
        
        lm.setPosition(linear_speed)
        
pass