from controller import Robot, Keyboard

robot = Robot()
keyboard = Keyboard()
TIMESTEP = 64

rm = robot.getDevice("rm")
rotate = 0.0


while robot.step(TIMESTEP) != -1:
    if rotate > 1:
        rotate += 0.05
        
    rm.setPosition(rotate)
        
pass