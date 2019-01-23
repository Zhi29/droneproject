import sys
import time

import navio.pwm
import navio.util
import datetime
import curses
import navio.rcinput

navio.util.check_apm()


rcin = navio.rcinput.RCInput()

#PWM SETTING######################################################
#PWM_OUTPUT = 1
PWM_OUTPUT_0  = 0
PWM_OUTPUT_1  = 1
PWM_OUTPUT_2  = 2
PWM_OUTPUT_3  = 3
SERVO_MIN = 1.072 #ms
SERVO_MAX = 1.935 #ms
SERVO_NOM = 1.500 #ms
SERVO_NOM_1 = 1.350
SERVO_STOP = 0.000

pwm0 = navio.pwm.PWM(PWM_OUTPUT_0)
pwm1 = navio.pwm.PWM(PWM_OUTPUT_1)
pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
pwm3 = navio.pwm.PWM(PWM_OUTPUT_3)
# pwm.initialize()
# pwm.set_period(50)
# pwm.enable()
pwm0.initialize()
pwm1.initialize()
pwm2.initialize()
pwm3.initialize()

pwm0.set_period(50)
pwm1.set_period(50)
pwm2.set_period(50)
pwm3.set_period(50)

pwm0.enable()
pwm1.enable()
pwm2.enable()
pwm3.enable()
##################################################################


def loop_for(seconds, func, *args):
    endTime = datetime.datetime.now() + datetime.timedelta(seconds=seconds)

    while True:
        if datetime.datetime.now() >= endTime:
            break
        func(*args)



def calibration_ESC():
    '''
    #pwm =  navio.pwm.PWM(PWM_OUTPUT)
    pwm0 = navio.pwm.PWM(PWM_OUTPUT_0)
    pwm1 = navio.pwm.PWM(PWM_OUTPUT_1)
    pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
    pwm3 = navio.pwm.PWM(PWM_OUTPUT_3)
   # pwm.initialize()
   # pwm.set_period(50)
   # pwm.enable()
    pwm0.initialize()
    pwm1.initialize()
    pwm2.initialize()
    pwm3.initialize()

    pwm0.set_period(50)
    pwm1.set_period(50)
    pwm2.set_period(50)
    pwm3.set_period(50)

    pwm0.enable()
    pwm1.enable()
    pwm2.enable()
    pwm3.enable()
'''
    print("set all pwm to 0")
    pwm0.set_duty_cycle(SERVO_STOP)
    pwm1.set_duty_cycle(SERVO_STOP)
    pwm2.set_duty_cycle(SERVO_STOP)
    pwm3.set_duty_cycle(SERVO_STOP)


    print("SERVO_MAX")
    loop_for(2, pwm0.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(4, pwm0.set_duty_cycle, SERVO_MIN)
    print("SERVO_MIN_AGAIN")
    loop_for(6, pwm0.set_duty_cycle, SERVO_MIN)


    print("SERVO_MAX")
    loop_for(2, pwm1.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(4, pwm1.set_duty_cycle, SERVO_MIN)
    print("SERVO_MIN_AGAIN")
    loop_for(6, pwm1.set_duty_cycle, SERVO_MIN)

    #pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
    #pwm2.initialize()
    #pwm2.set_period(50)
    #pwm2.enable()
    print("SERVO_MAX")
    loop_for(2, pwm2.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(4, pwm2.set_duty_cycle, SERVO_MIN)
    print("SERVO_MIN_AGAIN")
    loop_for(6, pwm2.set_duty_cycle, SERVO_MIN)

    print("SERVO_MAX")
    loop_for(2, pwm3.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(4, pwm3.set_duty_cycle, SERVO_MIN)
    print("SERVO_MIN_AGAIN")
    loop_for(6, pwm3.set_duty_cycle, SERVO_MIN)



   # print("SERVO_MIN_AGAIN")
   # loop_for(8, pwm.set_duty_cycle, SERVO_MIN)
   # print("SERVO_NOM")
   # loop_for(2, pwm.set_duty_cycle, SERVO_NOM_1)
   # print("SERVO_NOM_CHANGE")
   # loop_for(2, pwm.set_duty_cycle, SERVO_NOM)
   # print("Stop")
   # loop_for(2, pwm.set_duty_cycle, SERVO_STOP)

#calibration_ESC() 

def test_throttle():
    wait_until_motor_is_ready()
    i = 0.000
    Loop = True
    while Loop:

        print("i: ",i)
        SERVO_INPUT = 1.250 + i
        loop_for(0.01, pwm0.set_duty_cycle, SERVO_INPUT)
        loop_for(0.01, pwm1.set_duty_cycle, SERVO_INPUT)
        loop_for(0.01, pwm2.set_duty_cycle, SERVO_INPUT)
        loop_for(0.01, pwm3.set_duty_cycle, SERVO_INPUT)

        i = i + 0.050 

        if i >= 0.500:
            Loop = False
        time.sleep(0.5)

    loop_for(0.1, pwm0.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm1.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm2.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm3.set_duty_cycle, SERVO_MIN)

def test_throttle_via_RC():
    wait_until_motor_is_ready()
    Loop = True
    while Loop:
        period = float(rcin.read(2))/1000.000
        print("rcinput_value: ", period)
        loop_for(0.01, pwm0.set_duty_cycle, period)
        loop_for(0.01, pwm1.set_duty_cycle, period)
        loop_for(0.01, pwm2.set_duty_cycle, period)
        loop_for(0.01, pwm3.set_duty_cycle, period)





def wait_until_motor_is_ready():
    print("I am in the wait_until_motor_is_ready()")
    loop_for(0.1, pwm0.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm1.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm2.set_duty_cycle, SERVO_MIN)
    loop_for(0.1, pwm3.set_duty_cycle, SERVO_MIN)

    time.sleep(8)



calibration_ESC()
test_throttle_via_RC()

#test_throttle()



'''

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'): 
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            screen.addstr(0, 0, 'right')
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'left ')        
        elif char == curses.KEY_UP:
            screen.addstr(0, 0, 'up   ')        
        elif char == curses.KEY_DOWN:
            screen.addstr(0, 0, 'down ')
finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
'''