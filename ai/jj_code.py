#!/usr/bin/python3

import ev3dev.ev3 as ev3
import math
from time import sleep


#defines
CM_TICK = 145/3000 			#centimeter per tick
DEFAULT_SP = 350
SP_DIFF = 150
SP_TURN = 125
COL_R_THRESH = 50
COL_L_THRESH = 50

#global variables
state = "default"

#inputs
col_sens_r=ev3.ColorSensor('in4')	#The one with the red/black cap
col_sens_l=ev3.ColorSensor('in1')
btn = ev3.Button()



#outputs
mR = ev3.LargeMotor('outD')
mL = ev3.LargeMotor('outA')
mG = ev3.MediumMotor('outB')

def drive_cm(cm):
	motor_val = [mR.position,mL.position]
	while((mR.position-motor_val[0]+mL.position-motor_val[1])/2 < cm/CM_TICK):
		update_state()
		if(state == "default" or state == "reachLine"):
			set_speed(DEFAULT_SP,DEFAULT_SP)
		if(state == "left"):
			set_speed(DEFAULT_SP,DEFAULT_SP+SP_DIFF)
		if(state == "right"):
			set_speed(DEFAULT_SP+SP_DIFF,DEFAULT_SP)
	break_motors()
	return

def back_cm(cm):
	motor_val = [mR.position,mL.position]
	while((mR.position-motor_val[0]+mL.position-motor_val[1])/2 > cm/CM_TICK):
		update_state()
		if(state == "default" or state == "reachLine"):
			set_speed(-DEFAULT_SP,-DEFAULT_SP)
		if(state == "left"):
			set_speed(-DEFAULT_SP,-DEFAULT_SP+SP_DIFF)
		if(state == "right"):
			set_speed(-DEFAULT_SP+SP_DIFF,-DEFAULT_SP)
	break_motors()
	return


def close_gripper():
	global state
	mG.run_to_abs_pos(position_sp=-600, speed_sp=750, stop_action="hold")
	mG.wait_while('running')
	return

def open_gripper():
	mG.run_to_abs_pos(position_sp=0, speed_sp=750, stop_action="hold")
	mG.wait_while('running')
	return

def task_done():
	open_gripper()
	break_motors()
	return

def set_speed(speed_r, speed_l):
	mR.run_forever(speed_sp = speed_r)
	mL.run_forever(speed_sp = speed_l)
	return

def run_f():
	global state
	mR.run_forever(speed_sp=DEFAULT_SP)
	mL.run_forever(speed_sp=DEFAULT_SP)
	while(not (state == "reachLine")):
		update_state()
		if(state == "default"):
			set_speed(DEFAULT_SP,DEFAULT_SP)
		if(state == "left"):
			set_speed(DEFAULT_SP,DEFAULT_SP+SP_DIFF)
		if(state == "right"):
			set_speed(DEFAULT_SP+SP_DIFF,DEFAULT_SP)
	break_motors()
	state = "default"
	drive_cm(6)
	return

def run_l():
	global state
	while(state == "right"):
		update_state()
		set_speed(SP_TURN,-SP_TURN)
	while(state == "default" or state == "left"):
		update_state()
		set_speed(SP_TURN,-SP_TURN)
	while(state == "right"):
		update_state()
		set_speed(SP_TURN,-SP_TURN)
	break_motors
	return

def run_r():
	global state
	while(state == "left"):
		update_state()
		set_speed(-SP_TURN,SP_TURN)
	while(state == "default" or state == "right"):
		update_state()
		set_speed(-SP_TURN,SP_TURN)
	while(state == "left"):
		update_state()
		set_speed(-SP_TURN,SP_TURN)
	break_motors
	return

def break_motors():
	mR.stop(stop_action="hold")
	mL.stop(stop_action="hold")
	return

def update_state():
	global state
	if btn.any():    
		ev3.Sound.beep().wait()
		exit()
		break_motors()
		task_done()
	if(state == "default"):
		if(col_sens_r.value() < COL_R_THRESH):
			state = "left"
		if(col_sens_l.value() < COL_L_THRESH):
			state = "right"
	if(state == "right"):
		if(col_sens_l.value() > COL_L_THRESH):
			state = "default"
		if(col_sens_r.value() < COL_R_THRESH):
			state = "reachLine"
	if(state == "left"):
		if(col_sens_r.value() > COL_R_THRESH):
			state = "default"
		if(col_sens_l.value() < COL_L_THRESH):
			state = "reachLine"
	if(state == "reachLine"):
		if(col_sens_r.value() > COL_R_THRESH or col_sens_l.value() > COL_L_THRESH):
			state = "default"
	print("State: "+str(state))
	return

def print_sensors():
	print("Color right: "+str(col_sens_r.value()))
	print("Color left: "+str(col_sens_l.value()))
	print("Motor right: "+str(mR.position))
	print("Motor left: "+str(mL.position))
	print("Gripper: "+str(mG.position))
	return

def run(string):
	for elements in string:
		if btn.any():    
			ev3.Sound.beep().wait()
			exit()
			break_motors()
			task_done()
		if(elements == "f"):
			run_f()
		if(elements == "r"):
			run_r()
		if(elements == "l"):
			run_l()
		if(elements == "G"):
			run_gc()
		if(elements == "R"):
			run_rc()
		if(elements == 't'):
			run_t()
	break_motors()
	return

def run_t():
	run_r()
	run_r()
	return

def run_gc():
	global state
	state = "default"
	run_f()
	close_gripper()
	return

def run_rc():
	global state
	drive_cm(13)
	open_gripper()
	back_cm(-13)
	return

#main
mG.reset()
eight = "frfrflflflflfrfr"
eight_w_can = "GRlfrfrGRlfrfrGRrflflGRrflflGRrflflGRrflflGRlfrfrGRlfrfr"
#7:21
first_line_solution = "lfrffffrffffrGfRrflflGffRrflflGffRrflflGRlfffrfffrffflfflflGRlfrfrGfRrflflGffRrflflGRlfrfrGRrflflGfRtffrfffffrffflffflfffflflGRlfrffrGffRlfrfrGfRrflflGffRlfrfrGRlfrfrGRrflffffrfflffflfflflGRlfrfrGffRlfrfrGfRrflflGffRlfrffrffrfrGRrflflGR"
#7:10
jj_sol = "GRlfrfffrffffrGfRrflflGffRrflflGfRlflfrffrffflfflflGRtfrflfflflGfRlfrffrGfRrflflGffRlfrflGRrflflGRlffrflfflflGRlfffrffflfflflGRtffflfflflGRlfrfrGRlfrfffrGfRrfrfflfflflGfRlfrfrGRrflfflGfffRrflflGffRtfrGRlfrffffrffrfrGRrflflGffRrflfflfflflGRlfrfrGR"
#6:49
jj_sol_weighted = "GRlfrfffrffffrGfRrflflGffRrflflGfRlflfrffrffflfflflGRtfrflfflflGfRlfrffrGfRrflflGffRrflflGRlfffrffflfflflGRtffflfflflGRlfrfrGRlfrfffrGfRrfrfflfflflGfRrfflGffRrflfflfflGRlGRlfrfrGRrflfffrflfrffrfrGfRrflflGfffRlfGRrflflGRlfffflflGffR"
#0:00
jj_sol_re =
"GRlfrfffrffffrGfRrflflGffRrflflGffRrflflGRlffrflfrffrffflfflflGRlfrfrGfRrfrflfrflfflflGffRrfflGffRrflflGRlfffrflfrffrfrGfRrfrflffflfflflGRlfrfrGffRrfflGfffRtfffrflfrffrfrGfRrflflGffRrflfflGffRtfrGRlGRlfrfrGR"
#0:00
jj_sol_weighted_no_high_replace = "GRlfrfffrffffrGfRrflflGffRrflflGfRlflfrffrffflfflflGRtfrflfflflGfRlfrffrGfRrflflGffRrflflGRlfffrffflfflflGRtffflfflflGRlfrfrGfRlfrffrGfRrflflGffRrflfflfflGRlGRlfrfrGRrflfffrffflfflflGRlfrfrGfRrflflGfffRlfGRrflflGRlfffflflGffR"
#run(eight)
#close_gripper()

#for i in range(10):
#	run(eight_w_can)
	#print_sensors()
#	sleep(3)
run(first_line_solution)
task_done()
