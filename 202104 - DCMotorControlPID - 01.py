import serial
import tkinter as tk

comPort = "COM3"
ser = serial.Serial(comPort, baudrate = 9600, timeout = 0) #timeout is in seconds
ser2 = serial.Serial("COM4", baudrate = 9600, timeout = 0)
serialString = "" #initilizes string for serial port


def run(event):
    Position1 = DCMotorPosition1_Entry.get().encode() # .encode() converts the char type into byte types
    Position2 = DCMotorPosition2_Entry.get().encode()
    #print (type(InputTheta))
    x = b'<' + b'p' + b'1' + Position1 + b'>'+ b'<' + b'p' + b'2' + Position2 + b'>'
    ser.write(x)    
    print(x)
        
def blink():
    y = b'<'+ b'B'+ b'>'
    ser2.write(y)
    print(y)
    

root = tk.Tk() #this starts and creates the GUI

#-------------------------------------------------------------- Size and Title of GUI --------------------------------------------------------
root.geometry("700x500")
root.title('Continuum Robot GUI')

run_button = tk.Button(root, text ="run", command=run, font = 'sans 16 bold')
run_button.grid(row=0, column=0) 
root.bind("<Return>", run) #this allows you to just press enter to execute the "run"
LED_blink = tk.Button(root, text ="blink", command=blink, font = 'sans 16 bold')
LED_blink.grid(row=1,column=0)
#manual_button = tk.Button(root, text = "Manual Control On", command = manual, font = 'san 16 bold')
#manual_button.grid(row=1,column=0)
#manual_button_off = tk.Button(root, text = "Manual Control OFF", command = manual_off, font = 'san 16 bold')
#anual_button_off.grid(row=1,column=0)

DCMotorPosition1 = tk.Label(root, text = "Position 1 (steps): " , bd = 2)
DCMotorPosition1.grid(row=0,column=1)
DCMotorPosition1_Entry = tk.Entry(root, bd=1)
DCMotorPosition1_Entry.grid(row=0,column=2)

DCMotorPosition2 = tk.Label(root, text = "Position 2 (steps): " , bd = 2)
DCMotorPosition2.grid(row=1,column=1)
DCMotorPosition2_Entry = tk.Entry(root, bd=1)
DCMotorPosition2_Entry.grid(row=1,column=2)

root.mainloop() #This ends the GUI