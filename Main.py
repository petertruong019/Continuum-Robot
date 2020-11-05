from time import sleep
import RPi.GPIO as GPIO

DIR = 21
STEP = 20
DIR2 = 19
STEP2 = 13
CW = 1
CCW = 0
SPR = 200

#encoder pins
A1 = 23
B1 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.setup(A1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(B1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

step_count = SPR
delay = 0.005/12

counter = 0
last_state = GPIO.input(A1)

try:
    while True:
        GPIO.output(DIR, CW)
        GPIO.output(DIR2, CW)
        for x in range(0,step_count):
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP2, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            GPIO.output(STEP2, GPIO.LOW)
            sleep(delay)
    
        sleep(.25)

        GPIO.output(DIR, CCW)
        GPIO.output(DIR2, CCW)

        for x in range(0,step_count):
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP2, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            GPIO.output(STEP2, GPIO.LOW)
            sleep(delay)
        

        A1_state = GPIO.input(A1)
        B1_state = GPIO.input(B1)
        
        if (A1_state != last_state):
            if (B1_state != A1_state):
                counter += 1
                
            else:
                counter -= 1
            print (counter)
        last_state = A1_state
        sleep(0.005)



finally:
    GPIO.cleanup()