from controller import Robot, Keyboard

robot = Robot()
keyboard = Keyboard()
TIMESTEP = 64

lm = robot.getDevice("wrist_motor")
linear_speed = 0.0


while robot.step(TIMESTEP) != -1:
    if linear_speed > -2:
        linear_speed += 0.2
    else:
        break
        
pass