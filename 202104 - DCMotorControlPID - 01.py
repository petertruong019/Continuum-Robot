import serial
import tkinter as tk
import threading
import time
import os

comPort = "COM4" #Arduino 1 that controls the motors.
comPort2 = "COM8" #Arduino 2 that is preprogrammed to blink its LED.
ser = serial.Serial(comPort, baudrate = 9600, timeout = 0) #timeout is in seconds
ser2 = serial.Serial(comPort2, baudrate = 9600, timeout = 0)
#serialData = ''
#ser.flushInput()

def run():
    Position1 = DCMotorPosition1_Entry.get().encode() # .encode() converts the char type into byte types
    Position2 = DCMotorPosition2_Entry.get().encode()
    x = b'<' + b'p' + b'1' + Position1 + b'>'+ b'<' + b'p' + b'2' + Position2 + b'>'
    ser.write(x)    
    print(x)
        
def blink():
    
    y = b'<'+ b'B'+ b'>'
    ser2.write(y)
    #print(y)
    
def updatelabel():
    serialData = ser.readline()
    StringDecode = serialData.decode('Ascii')
    label1=tk.Label(root, text = StringDecode)
    label1.grid(row=3,column=2)
    root.after(20,updatelabel)

root = tk.Tk() #this starts and creates the GUI
#-------------------------------------------------------------- Size and Title of GUI --------------------------------------------------------
root.geometry("700x500")
root.title('Continuum Robot GUI')
run_button = tk.Button(root, text ="run", command=run, font = 'sans 16 bold')
run_button.grid(row=0, column=0) 
#root.bind("<Return>", run) #this allows you to just press enter to execute the "run"

LED_blink = tk.Button(root, text ="blink", command=blink, font = 'sans 16 bold')
LED_blink.grid(row=1,column=0)

DCMotorPosition1 = tk.Label(root, text = "Position 1 (steps): " , bd = 2)
DCMotorPosition1.grid(row=0,column=1)
DCMotorPosition1_Entry = tk.Entry(root, bd=1)
DCMotorPosition1_Entry.grid(row=0,column=2)

DCMotorPosition2 = tk.Label(root, text = "Position 2 (steps): " , bd = 2)
DCMotorPosition2.grid(row=1,column=1)
DCMotorPosition2_Entry = tk.Entry(root, bd=1)
DCMotorPosition2_Entry.grid(row=1,column=2)

label1=tk.Label(root, text = "label")
label1.grid(row=3,column=2)


updatelabel()
root.mainloop() #This ends the GUI