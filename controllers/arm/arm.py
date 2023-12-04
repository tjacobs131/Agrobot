from controller import Robot, Keyboard

robot = Robot()
keyboard = Keyboard()
TIMESTEP = 64

lm = robot.getDevice("linear_motor")
linear_speed = 0.0

while robot.step(TIMESTEP) != -1:  # Main control loop
    # Check for keyboard events
    key = keyboard.getKey()
    
    if key == 87 and linear_speed < 5:
        # Moves up
        linear_speed += 0.2
        print(linear_speed)
    elif key == 83 and linear_speed > -2:
        # Moves down
        linear_speed -= 0.2
        print(linear_speed)
    else:
        linear_speed += 0
        
    lm.setPosition(linear_speed)    
        
pass