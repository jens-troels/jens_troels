#Original author: Jens Troels Nielsen

import simpleguitk as simplegui
from math import sin, pi, cos, copysign

#########################################################
# User Functions
def deg2rad(degree):
    rad = degree*pi/180
    return rad

#########################################################
# Pendulum and Simulation variables

l1		 	= 1						# Length of the pendulum shoulder[m]
l2			= 1						# Length of the pendulum elbow[m]
lc1			= l1/2				 		# Length to center of link [m]
lc2			= l2/2						# Length to center of link [m]
m1 			= 1						# Mass of pendulum shoulder [kg]
m2			= 1						# Mass of pendulum elbow [kg]
g	 		= 9.82						# Gravitational acceleration [m/s/s]
start_angle 		= deg2rad(50)				        # Store initial angle [rad]
start_angle2            = deg2rad(10)                                  # Store initial angle2 [rad]
theta1 			= start_angle					# Initial angle [rad]
theta2			= start_angle2					# Initial angle2 [rad]
omega1         		= 0						# Initial velocity [rad/s]
omega2			= 0
alpha1 		        = 0.0						# Initial acceleration [rad/s/s]
alpha2			= 0
I1			= 1			                        # Inertia of pendulum shoulder
I2			= 1					        # Inertia of pendulum elbow
tau			= 0						# Torque [N*m]
K1			= 9.1630*1000                                   # LQR gains for different states
K2			= 3.6389*1000
K3			= 3.6984*1000
K4			= 1.4212*1000					
energy			= 0						# Energy of the system
state			= 0						# State defining controller
oldErr			= 0

CANVAS_WIDTH	 	= l1*100*5				        # Width of the simulation area
CANVAS_HEIGHT	 	= l1*100*5				        # Height of the simulation area
TIME_STEP 		= 0.001       				        # How many seconds between updates
COLOR			= 'Green'

f = open("acro.dat","w+")

#########################################################
# Controller (IMPLEMENT YOUR CONTROLLER HERE)
# U = controller output
   
class controllerClass(object):
    def __init__(self):
        self.u = 0
        
    def update(self):		
        global z
	global state
        global oldErr
	
        if state == 0:
            z = (energy-38.29)*omega1
	    
        if state == 1:
            z = theta2*100+(theta2-oldErr)*500
            oldErr = theta2
        
	if state == 2:
            z = K1 * (theta1-deg2rad(180)) + K2 * omega1 + K3 * theta2 + K4 * omega2	# LQR controller
	
	
        self.u = -z

    def getU(self):
        return self.u  

#########################################################
# Button handler functions
def reset():
    global alpha1, alpha2, omega1, omega2, theta1, theta2, state, tau
    omega1 = 0
    omega2 = 0
    theta1 = start_angle
    theta2 = start_angle2
    alpha1 = 0
    alpha2 = 0
    tau = 0
    state = 0
    
def quit():
    f.close()
    exit()
    
#########################################################
# Event handler functions

def potentialEnergy(theta1, theta2):
    return -m1*g*lc1*(1-cos(theta1))-m2*g*(l1*(1-cos(theta1))+l2*(1-cos(theta1+theta2)))

def kineticEnergy(theta1, theta2, omega1, omega2):
    T1 = 0.5*I1*omega1**2
    T2 = 0.5*(m2*l1**2+2*m2*l1*lc2*cos(theta2))*omega1**2+0.5*I2*omega2**2+(I2+m2*l1*lc2*cos(theta2))*omega1*omega2
    return T1+T2

def drawAcrobot(canvas):
    '''
    Event handler for redrawing the canvas
    '''
    nAngle = theta1
    nAngle2 = theta2

    canvas.draw_line( (CANVAS_WIDTH/2, CANVAS_HEIGHT/2) , (CANVAS_WIDTH/2+l1*50*sin(nAngle),CANVAS_HEIGHT/2+l1*50*cos(nAngle)), 5, "Red")
    canvas.draw_line( (CANVAS_WIDTH/2+l1*50*sin(nAngle),CANVAS_HEIGHT/2+l1*50*cos(nAngle)) , (CANVAS_WIDTH/2+(l1*50*sin(nAngle))+(l2*50*sin(nAngle2+nAngle)),CANVAS_HEIGHT/2+(l1*50*cos(nAngle))+(l2*50*cos(nAngle2+nAngle))), 5, "Blue")
    canvas.draw_circle((CANVAS_WIDTH/2+l1*50*sin(nAngle),CANVAS_HEIGHT/2+l1*50*cos(nAngle)), 3, 3, "Green", "Green")
    canvas.draw_circle((CANVAS_WIDTH/2,CANVAS_HEIGHT/2), 3, 3, "Black", "Black")

def update():
    '''
    Event handler for timer based simulation updates
    '''
    global alpha1, omega1, theta1, alpha2, omega2, theta2, tau, state

    controller.update()
    tau = controller.getU()


    alpha1 = -(((I2 + l1*lc2*m2*cos(theta2))*(tau + ((I2 + l1*lc2*m2*cos(theta2))*(- l1*lc2*m2*sin(theta2)*omega2**2 - 2*l1*lc2*m2*omega1*sin(theta2)*omega2 + g*sin(theta1)*(l1*m2 + lc1*m1) + g*l2*m2*sin(theta1 + theta2)))/(m2*l1**2 + 2*lc2*m2*cos(theta2)*l1 + I1 + I2) - g*l2*m2*sin(theta1 + theta2) - l1*lc2*m2*omega1**2*sin(theta2)))/(I2 - (I2 + l1*lc2*m2*cos(theta2))**2/(m2*l1**2 + 2*lc2*m2*cos(theta2)*l1 + I1 + I2)) + g*sin(theta1)*(l1*m2 + lc1*m1) + g*l2*m2*sin(theta1 + theta2) - l1*lc2*m2*omega2**2*sin(theta2) - 2*l1*lc2*m2*omega1*omega2*sin(theta2))/(m2*l1**2 + 2*lc2*m2*cos(theta2)*l1 + I1 + I2)


    alpha2 = (tau + ((I2 + l1*lc2*m2*cos(theta2))*(- l1*lc2*m2*sin(theta2)*omega2**2 - 2*l1*lc2*m2*omega1*sin(theta2)*omega2 + g*sin(theta1)*(l1*m2 + lc1*m1) + g*l2*m2*sin(theta1 + theta2)))/(m2*l1**2 + 2*lc2*m2*cos(theta2)*l1 + I1 + I2) - g*l2*m2*sin(theta1 + theta2) - l1*lc2*m2*omega1**2*sin(theta2))/(I2 - (I2 + l1*lc2*m2*cos(theta2))**2/(m2*l1**2 + 2*lc2*m2*cos(theta2)*l1 + I1 + I2))


    omega1 = omega1 + (alpha1*TIME_STEP)
    omega2 = omega2 + (alpha2*TIME_STEP)
    theta1 = theta1 + (omega1*TIME_STEP)
    theta2 = theta2 + (omega2*TIME_STEP)

    print("Vinkel1: "+str(theta1)+" Vinkel2: "+str(theta2))


    #wrap around
    
    if theta1>deg2rad(360):
        theta1 = theta1 - deg2rad(360)
    elif theta1<deg2rad(0):
        theta1 = theta1 + deg2rad(360)
    if theta2>deg2rad(180):
        theta2 = theta2 - deg2rad(360)
    elif theta2<deg2rad(-180):
        theta2 = theta2 + deg2rad(360)
    '''
    if omega1>10*pi:
        omega1=10*pi
    elif omega1<-10*pi:
        omega1=-10*pi
    if omega2>10*pi:
        omega2=10*pi
    elif omega2<-10*pi:
        omega2=-10*pi
    '''
    

    energy = -potentialEnergy(theta1, theta2)+kineticEnergy(theta1, theta2, omega1, omega2)

    
    if energy>=106 and state == 0:
	state = 1
    if  state == 1 and abs(theta1) < deg2rad(180+10) and abs(theta1) > deg2rad(180-10) and abs(theta2) < deg2rad(2) and energy < 50:
        state = 2
    

    print("pot: "+str(potentialEnergy(theta1, theta2))+" kin: "+str(kineticEnergy(theta1, theta2, omega1, omega2))+" tot: "+str(energy))
    print(state)

    f.write("%f\t" % theta1)
    f.write("%f\t" % theta2)
    f.write("%f\t" % omega1)
    f.write("%f" % omega2)
    f.write("\n")

#########################################################
# Simulation script:

frame = simplegui.create_frame("Acrobot simulator", CANVAS_WIDTH, CANVAS_HEIGHT)
frame.set_draw_handler(drawAcrobot)
frame.set_canvas_background("White")

# Create timers
timer = simplegui.create_timer(TIME_STEP*1000, update)

# Register event handlers
# frame.add_button("Add robot", addRobotButtonClick, 100)
frame.add_button("Reset", reset, 100)
frame.add_button("Quit", quit, 100)

# Create controller
controller = controllerClass()

# Start 
#reset()
timer.start()
frame.start()

