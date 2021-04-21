# GUI_v3a.py
'''
changes made:
- add kalman filter this one is hard
- add tabs
'''

import time
import smbus
import math
import matplotlib.pyplot as plt
import serial
import board
 
from tkinter import *
from tkinter import ttk
import tkinter as tk
import tkinter.font
import tkinter.messagebox as MessageBox
import RPi.GPIO # for cleanup
RPi.GPIO.setmode(RPi.GPIO.BCM)

from adafruit_mpu6050 import MPU6050

newx_position = 0 
newy_position = 0   
 
### HARDWARE ### 
# I2C - IMU
blossom_mpu6050 = MPU6050(board.I2C())           # base accel & gyro sensor
# Power management registers - MPU6050
# power_mgmt_1 = 0x6b
# power_mgmt_2 = 0x6c
 
# gyro_scale = 131.0
# accel_scale = 16384.0
 
# address = 0x68  # This is the address value read via the i2cdetect command

# Serial - Jevois
# jevois_baudrate= 115200
# com_port1 = '/dev/serial0'
# ser1 = serial.Serial(port = com_port1, baudrate = jevois_baudrate,
#                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
#                     bytesize=serial.EIGHTBITS, timeout=1)

# Serial - Arduino
# arduino_baudrate = 115200 
# com_port2 = '/dev/ttyACM0'    # under the wifi usb
# ser2 = serial.Serial(port = com_port2, baudrate = arduino_baudrate, timeout = 0)    # my port = '/dev/ttyACM0'

### GUI DEFINITIONS ###
HEIGHT = 700   # pixels
WIDTH = 1300

root = Tk()                                      # create Tkinter root
root.title("Continuum Robot GUI")

canvas = tk.Canvas(root, height=HEIGHT, width=WIDTH)
canvas.pack

my_notebook = ttk.Notebook(root)
my_notebook.pack(pady = 0)

myFont = tkinter.font.Font(family = 'Helvetica',
                           size = 12,
                           weight = "bold")

# define what is inside the tabs using frames
mode_selection_frame = Frame(my_notebook, bd = 10,
                             width = HEIGHT, height = WIDTH, 
                             bg = '#80c1ff')
manual_mode_frame = Frame(my_notebook, bd = 10,
                          width = HEIGHT, height = WIDTH, 
                          bg = '#80c1ff')
object_tracing_mode_frame = Frame(my_notebook, bd = 10,
                                  width = HEIGHT, height = WIDTH,
                                  bg = '#80c1ff')
pattern_mode_frame = Frame(my_notebook, bd = 10,
                           width = HEIGHT, height = WIDTH,
                           bg = '#80c1ff')

mode_selection_frame.pack(fill = 'both', expand = 1)
manual_mode_frame.pack(fill = 'both', expand = 1)
object_tracing_mode_frame.pack(fill = 'both', expand = 1)
pattern_mode_frame.pack(fill = 'both', expand = 1)

# designate the tabs
my_notebook.add(mode_selection_frame, text = 'Mode Selection')
my_notebook.add(manual_mode_frame, text = 'Manual Mode')
my_notebook.add(object_tracing_mode_frame, text = 'Object Tracing Mode')
my_notebook.add(pattern_mode_frame, text = 'Pattern Mode')

# hide the different modes
my_notebook.hide(1)
my_notebook.hide(2)
my_notebook.hide(3)

### EVENT FUNCTIONS ###
running = True # global flag

def close_window():
    root.destroy()

def send_to_jevois_program(cmd):
    """Send commands to the Jevois program to control the camera

    Args:
        cmd ([string]): the command to be sent to the jevois program terminal
    """
    # print(cmd)
    ser1.write((cmd + '\n').encode())
    time.sleep(1)
    print('Message was sent to Jevois!')
  
def move_motors(x, y):
    global newx_position, newy_position
    
    pulse_position1 = str(x*2 + int(newx_position))
    pulse_position2 = str(y*2 + int(newy_position))
    
    newx_position = pulse_position1
    newy_position = pulse_position2
    
    move_to_position1 = b'<' + b'p' + b'1' + pulse_position1.encode() + b'>'
    move_to_position2 = b'<' + b'p' + b'2' + pulse_position2.encode() + b'>'
    print('move to position 1:', move_to_position1)
    print('move to position 2:', move_to_position2)
    serialread2 = ser2.readline()
    print(serialread2)
    ser2.write(move_to_position1 + move_to_position2)

# def round_float(imu_reading):
#     round_to_decimal = 2
#     round_float.imu_reading = round(imu_reading, round_to_decimal)
#     # return imu_reading
#     # print("imu reading:", imu_reading)

def blink():
    y = b'<'+ b'B'+ b'>'
    ser2.write(y)

def run():
    if running:
        # blossom - base
        round_to_decimal = 2
        blossom_accel = blossom_mpu6050.acceleration    # reads blossom accel, tuple
        blossom_gyro = blossom_mpu6050.gyro             # reads blossom gyro, tuple
        accel_x, accel_y, accel_z = blossom_accel       # unpacks tuple
        gyro_x, gyro_y, gyro_z = blossom_gyro           
        
        accel_x = round(accel_x, round_to_decimal)                   # rounds float to 2 decimal places
        accel_y = round(accel_y, round_to_decimal)
        accel_z = round(accel_z, round_to_decimal)
        
        gyro_x = round(gyro_x, round_to_decimal)
        gyro_y = round(gyro_y, round_to_decimal)
        gyro_z = round(gyro_z, round_to_decimal)

        blossom_accel_string = "Acceleration: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} m/s^2".format(*blossom_accel)
        blossom_gyro_string = "Amngular Velocity: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} rad.s".format(*blossom_gyro)
        blossom_Atext.set(blossom_accel_string)
        blossom_Gtext.set(blossom_gyro_string)

        time.sleep(0.5)
        
        # Jevois Reading
        ser1.flushInput()

        serialread = ser1.readline().rstrip().decode('utf8')
        data_list = serialread.split('x')
        list_check = str(data_list)
        no = "['']"
                
        if list_check != no:
            #using map() to turn string array into int array
            data_list = list(map(int, data_list))
            x = data_list[0]     # int
            y = data_list[1]     # int
                    
            print ('X coordinate: {} | Y coordinate: {}'.format(x,y))
            jevois_reading = 'X coordinate: {} | Y coordinate: {}'.format(x,y)
            jevois_text.set(jevois_reading)
            if x != 0:
                move_motors(x, y)
                # blink()
                
    if not running:
        print("this is not running")
        buttercup_text.set("buttercup IMU readings unavailable")
        bubbles_text.set("Bubbles IMU readings unavailable")
 
    # after 0.5 s, call scanning again,  1/2 s = 500
    root.after(500, run)
    
def show_tab(mode_frame, mode_selection):
    my_notebook.add(mode_frame, text = mode_selection)
    
def close_tab(i_tab):
    my_notebook.hide(i_tab)
    
### WIDGET FUNCTIONS ###
def something(RELX, RELY, RELWIDTH, RELHEIGHT):
    position_label = Label(manual_mode_frame, text = 'position (pulses)', font = myFont)
    position_label.place(relx = RELX, rely = RELY, 
                     relwidth = RELWIDTH, relheight = RELHEIGHT)
 
    
### WIDGETS ###
# GLOBAL MODE TAB WIDGET SIZING #
title_rel_height = 0.05
title_rel_width = 0.8
close_tab_relx = 0.95
close_rel_height = 0.05
close_rel_width = 0.05

# MODE SELECTION TAB #
mode_sel_rely = 0.2
mode_sel_relheight = 0.05
mode_sel_relwidth = 0.2

select_mode_label = Label(mode_selection_frame, text = 'Select Control Mode:', 
                          font = myFont, bg = 'white')
select_mode_label.place(relx = 0.5,
                        relheight = title_rel_height, relwidth = title_rel_width,
                        anchor = 'n')

manual_button = Button(mode_selection_frame, text = 'Manual Mode',
                       font = myFont, 
                       command = lambda: show_tab(manual_mode_frame, 'Manual Mode'))
manual_button.place(relx = 0.15, rely = mode_sel_rely,
                    relheight = mode_sel_relheight, relwidth = mode_sel_relwidth)
object_trace_button = Button(mode_selection_frame, text = 'Object Tracing Mode',
                             font = myFont,
                             command = lambda: show_tab(object_tracing_mode_frame, 'Object Tracing Mode'))
object_trace_button.place(relx = 0.4, rely = mode_sel_rely,
                          relheight = mode_sel_relheight, relwidth = mode_sel_relwidth)
pattern_button = Button(mode_selection_frame, text = 'Pattern Mode',
                        font = myFont,
                        command = lambda: show_tab(pattern_mode_frame, 'Pattern Mode'))
pattern_button.place(relx = 0.65, rely = mode_sel_rely,
                     relheight = mode_sel_relheight, relwidth = mode_sel_relwidth)

manual_description_text = 'Manual Mode: Control the continuum robot manually by inputting the amount of pulses for the motors to move.'
object_description_text = 'Object Tracing Mode: <add description later>.'
pattern_description_text = 'Pattern Tracing Mode: <add description later>.'

mode_descriptions_text = ('{:<} \n \n {:<} \n \n {:<}'.format(manual_description_text, object_description_text, pattern_description_text))
# mode_descriptions_text = mode_descriptions_text + manual_description_text + object_description_text + pattern_description_text

# txt = "We have {:<} chickens."
# print(txt.format(49))

mode_descriptions = Label(mode_selection_frame, text = mode_descriptions_text,
                          font = myFont, bg = 'white',
                          anchor = 'w', justify = LEFT,
                          wraplength = 760)
mode_descriptions.place(relx = 0.5, rely = 0.35,
                        relheight = 0.3, relwidth = title_rel_width,
                        anchor = 'n')

exit_gui_button = tk.Button(mode_selection_frame, text = "Exit GUI", command = close_window)
exit_gui_button.place(rely = 0.9,
                      relheight = 0.1, relwidth = 0.2)

# MANUAL MODE TAB #
manual_title = Label(manual_mode_frame, text = 'Manual Mode',
                          font = myFont, bg = 'white')
manual_title.place(relx = 0.5,
                   relheight = title_rel_height, relwidth = title_rel_width,
                   anchor = 'n')
close_manual = Button(manual_mode_frame, text = 'X',
                      fg = 'white', bg = 'red',
                      font = myFont,
                      command = lambda: close_tab(1))
close_manual.place(relx = close_tab_relx,
                   relheight = close_rel_height, relwidth = close_rel_width)

# add a label that summarizes each mode?

# MOTOR CONTROL, a stands for arduino
ac1_relx = 0.1        # column 1: motor labels
ac2_relx = 0.3        # column 2: input position change
ac3_relx = 0.5        # column 3: current position labels
ac4_relx = 0.7        # column 4: encoder feedback

ar1_rely = 0.1        # enter position / new position / encoder feedback
ar2_rely = 0.225      # row 1: motor 1 info
ar3_rely = 0.35       # row 2: motor 2 info
ar4_rely = 0.475
  

ac0_relwidth = 0.15   # only column 1 has a different relwidth
a_relheight = 0.075   # everything has the same height
a_relwidth = 0.15      # everything has the same width


something(ac2_relx, ar1_rely, a_relwidth, a_relheight)
# position_label = Label(manual_mode_frame, text = 'position (pulses)', font = myFont)
# position_label.place(relx = ac2_relx, rely = ar1_rely, 
#                      relwidth = a_relwidth, relheight = a_relheight)

current_position_label = Label(manual_mode_frame, text = 'current position', font = myFont)
current_position_label.place(relx = ac3_relx, rely = ar1_rely,
                             relwidth = a_relwidth, relheight = a_relheight)

encoder_label = Label(manual_mode_frame, text = 'encoder feedback', font = myFont)

encoder_label.place(relx = ac4_relx, rely = ar1_rely, 
                    relwidth = a_relwidth, relheight = a_relheight)

motor1_label = Label(manual_mode_frame, text = 'motor 1:', font = myFont)
motor1_label.place(relx = ac1_relx, rely = ar2_rely, 
                   relwidth = ac0_relwidth, relheight = a_relheight)

motor2_label = Label(manual_mode_frame, text = 'motor 2:', font = myFont)
motor2_label.place(relx = ac1_relx, rely = ar3_rely, 
                   relwidth = ac0_relwidth, relheight = a_relheight)

position_motor1_txt = StringVar()
position_motor1_txt.set('')
position_motor2_txt = StringVar()
position_motor2_txt.set('')

# enter_position_motor1 = Entry(manual_mode_frame, textvariable = position_motor1_txt)
# enter_position_motor1.place(relx = ac2_relx, rely = ar2_rely, 
#                             relwidth = a_relwidth, relheight = a_relheight)

enter_position_motor2 = Entry(manual_mode_frame, textvariable = position_motor2_txt)
enter_position_motor2.place(relx = ac2_relx, rely = ar3_rely,
                            relwidth = a_relwidth, relheight = a_relheight)

position_motor1_txt.trace("w",lambda *args: print (position_motor1_txt.get()))
position_motor2_txt.trace("w",lambda *args: print (position_motor2_txt.get()))

current_position_motor1 = tk.Label(manual_mode_frame, textvariable = position_motor1_txt, 
                                   font = myFont, bg = 'white')
current_position_motor1.place(relx = ac3_relx, rely = ar2_rely, 
                              relwidth = a_relwidth, relheight = a_relheight)

current_position_motor2 = tk.Label(manual_mode_frame, textvariable = position_motor2_txt, 
                                   font = myFont, bg = 'white')
current_position_motor2.place(relx = ac3_relx, rely = ar3_rely, 
                              relwidth = a_relwidth, relheight = a_relheight)

encoder1_label = tk.Label(manual_mode_frame, font = myFont, bg = 'white')
                      #command = lambda: send_to_jevois_program('obstacle'))
encoder1_label.place(relx = ac4_relx, rely = ar2_rely, 
                     relwidth = a_relwidth, relheight = a_relheight)

encoder2_label = tk.Label(manual_mode_frame, font = myFont, bg = 'white')
                      #command = lambda: send_to_jevois_program('obstacle'))
encoder2_label.place(relx = ac4_relx, rely = ar3_rely, 
                     relwidth = a_relwidth, relheight = a_relheight)

run_button = Button(manual_mode_frame, text = "Run",
                    bg = 'green', font = myFont)
                      #command = lambda: send_to_jevois_program('obstacle'))
run_button.place (relx = 0.5, rely = ar4_rely, 
                  anchor = 'n',
                  relwidth = a_relwidth, relheight = a_relheight)

manual_exit_gui_button = tk.Button(manual_mode_frame, text = "Exit GUI", 
                                   command = close_window)
manual_exit_gui_button.place(rely = 0.9,
                             relwidth = 0.2, relheight = 0.1)

# OBJECT TRACING TAB #
object_title = Label(object_tracing_mode_frame, text = 'Object Tracing Mode',
                     font = myFont, bg = 'white')
object_title.place(relx = 0.5,
                   relheight = title_rel_height, relwidth = title_rel_width,
                   anchor = 'n')
close_obj_tracing = Button(object_tracing_mode_frame, text = 'X',
                           fg = 'white', bg = 'red',
                           font = myFont,
                           command = lambda: close_tab(2))
close_obj_tracing.place(relx = close_tab_relx,
                        relheight = close_rel_height, relwidth = close_rel_width)

object_exit_gui_button = tk.Button(object_tracing_mode_frame, text = "Exit GUI", 
                                   command = close_window)
object_exit_gui_button.place(rely = 0.9,
                             relwidth = 0.2, relheight = 0.1)

# PATTERN TAB #
pattern_title = Label(pattern_mode_frame, text = 'Pattern Mode',
                     font = myFont, bg = 'white')
pattern_title.place(relx = 0.5,
                    relheight = title_rel_height, relwidth = title_rel_width,
                    anchor = 'n')
close_pattern = Button(pattern_mode_frame, text = 'X',
                       fg = 'white', bg = 'red',
                       font = myFont,
                       command = lambda: close_tab(3))
close_pattern.place(relx = close_tab_relx,
                    relheight = close_rel_height, relwidth = close_rel_width)

pattern_exit_gui_button = tk.Button(pattern_mode_frame, text = "Exit GUI", 
                                   command = close_window)
pattern_exit_gui_button.place(rely = 0.9,
                             relwidth = 0.2, relheight = 0.1)

'''
start_button = Button(root, text = 'Display IMU Reading',
                      font = myFont, command = start,
                      bg = 'bisque2', height = 1,
                      width = 24)
start_button.grid(row=0, column=1) 
start_button.config(relief='raised')
 
stop_button = Button(root, text = 'Hide IMU Reading',
                     font = myFont, command = stop,
                     bg = 'bisque2', height = 1,
                     width = 24)
stop_button.grid(row=0, column=2)                 # location in gui
stop_button.config(relief='raised')
'''

'''
## Jevois - CV ##
# j = jevois, row = r, column = c
jr1_rely = 0.3
jr2_rely = 0.7
jc1_relx = 0.25
jc2_relx = 0.5
jc3_relx = 0.75
j_relheight = 0.3
j_relwidth = 0.2

jevois_label = tk.Label(upper_frame, text = 'Computer Vision Control', font = myFont)
jevois_label.place(relx = 0.5, 
                   relheight = 0.25, relwidth = 0.25,
                   anchor = 'n')

mode_label = tk.Label(upper_frame, text = 'select mode:', font = myFont, bg = 'white')
mode_label.place(rely = jr1_rely, 
                 relheight = j_relheight, relwidth = j_relwidth)

color_label = tk.Label(upper_frame, text='select color:', font = myFont, bg = 'white')
color_label.place(rely = jr2_rely, 
                  relheight = j_relheight, relwidth = j_relwidth)

jevois_text = StringVar()
jevois_text.set("Jevois Camera Readings")
jevois_output = Label(lower_frame, textvariable = jevois_text, bg = 'white')
jevois_output.place(rely = 0.6,
                    relheight = 0.15, relwidth = 1)

calibration_button = Button(upper_frame, text = 'calibration', font = myFont,
                            command = lambda: send_to_jevois_program('calibration'))
calibration_button.place(relx = jc1_relx, rely = jr1_rely, 
                         relheight = j_relheight, relwidth = j_relwidth)

obstacle_button = Button(upper_frame, text = 'obstacle', font = myFont,
                         command = lambda: send_to_jevois_program('obstacle'))
obstacle_button.place(relx = jc2_relx, rely = jr1_rely, 
                      relheight = j_relheight, relwidth = j_relwidth)

target_button = Button(upper_frame, text = 'target', font = myFont,
                       command = lambda: send_to_jevois_program('target'))
target_button.place(relx = jc3_relx, rely = jr1_rely, 
                    relheight = j_relheight, relwidth = j_relwidth)

red_button = Button(upper_frame, text = 'red', font = myFont, 
                    activebackground = 'red',
                    command = lambda: send_to_jevois_program('red'))
red_button.place(relx = jc1_relx, rely = jr2_rely, 
                 relheight = j_relheight, relwidth = j_relwidth)

green_button = Button(upper_frame, text = 'green', font = myFont, 
                      activebackground = 'green',
                      command = lambda: send_to_jevois_program('green'))
green_button.place(relx = jc2_relx, rely = jr2_rely, 
                   relheight = j_relheight, relwidth = j_relwidth)

blue_button = Button(upper_frame, text = 'blue', font = myFont, 
                     activebackground = 'blue',
                     command = lambda: send_to_jevois_program('blue'))
blue_button.place(relx = jc3_relx, rely = jr2_rely, 
                  relheight = j_relheight, relwidth = j_relwidth)

## Arduino - Motors ##

ar4_rely =  0.6       # row 3: motor 3 info
ar5_rely = 0.75       # row 4: motor 4 info
ar6_rely = 0.9        # row 5: buttons row
ac3_relx = 0.7        # column 3: encoder feedback
stepper_enable_relx = 0.63
run_relx = 0.85
run_relwidth = 0.1
warning_relwidth = 0.25


arduino_label = tk.Label(middle_frame, text = 'Arduino Communication', font = myFont)
arduino_label.place(relx = 0.5, 
                    relwidth = 0.25, relheight = a_relheight, 
                    anchor = 'n')



speed_label = tk.Label(middle_frame, text = 'speed (steps/s)', font = myFont)
speed_label.place(relx = ac2_relx, rely = ar1_rely, 
                  relwidth = a_relwidth, relheight = a_relheight)

speed_warning_label = tk.Label (middle_frame, text = 'warning: do not exceed 200 steps/s', font = myFont)
speed_warning_label.place(rely = ar6_rely, 
                          relwidth = warning_relwidth, relheight = a_relheight)


motor1_speed_entry = tk.Entry(middle_frame)
motor1_speed_entry.place(relx = ac2_relx, rely = ar2_rely, 
                         relwidth = a_relwidth, relheight = a_relheight)

motor2_speed_entry = tk.Entry(middle_frame)
motor2_speed_entry.place(relx = ac2_relx, rely = ar3_rely, 
                         relwidth = a_relwidth, relheight = a_relheight)


stepper_enable_button = Button(middle_frame, text = 'Stepper Motors Disabled!', 
                               bg = 'red', font = myFont)
                            #    command = stepper_enable)
stepper_enable_button.place(relx = stepper_enable_relx, rely = ar6_rely, 
                            relwidth = a_relwidth, relheight = a_relheight)



## IMU Readings ##
imu_label = tk.Label(lower_frame, text = 'IMU Readings',
                     font = myFont)
imu_label.place(relx = 0.5, 
                relwidth = 0.25, relheight = 0.17,
                anchor = 'n')

imu_accel_label = tk.Label(lower_frame, text = 'Acceleration',
                           font = myFont)
imu_accel_label.place(relx = 0.25, rely = 0.22,
                      relwidth = 0.3, relheight = 0.15)

imu_angular_velocity_label = tk.Label(lower_frame, text = 'Angular Velocity',
                                      font = myFont)
imu_angular_velocity_label.place(relx = 0.6, rely = 0.22, 
                                 relwidth = 0.3, relheight = 0.15)

base_imu_label = tk.Label(lower_frame, text = 'base IMU readings:',
                          font = myFont, bg = 'white')
base_imu_label.place(rely = 0.4,
                     relwidth = 0.2, relheight = 0.15)

blossom_Atext = StringVar()
blossom_Atext.set("Base Acceleration Data")
blossom_Aoutput = Label(lower_frame, textvariable = blossom_Atext)
blossom_Aoutput.place(relx = 0.25, rely = 0.4,
                      relheight = 0.15, relwidth = 0.3)

blossom_Gtext = StringVar()
blossom_Gtext.set("Base Angular Velocity Data")
blossom_Goutput = Label(lower_frame, textvariable = blossom_Gtext)
blossom_Goutput.place(relx = 0.6, rely = 0.4,
                      relheight = 0.15, relwidth = 0.3)
'''

# root.after(1000, run) # after 1 s, call scanning
root.mainloop()