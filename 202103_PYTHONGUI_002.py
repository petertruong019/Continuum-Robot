import serial
import tkinter as tk

comPort = "COM4"
ser = serial.Serial(comPort, baudrate = 115200, timeout = 0) #timeout is in seconds
serialString = "" #initilizes string for serial port
global stepper_enable_state
stepper_enable_state = False
global endeffector_enable_state
endeffector_enable_state = False

#----------------------------------------------------------------------- FUNCTIONS ----------------------------------------------------------------

def run():
    if (stepper_enable_state == True): #only runs the stepper motors if the stepperEnable Button is enabled
        InputTheta1 = theta_entry1.get().encode() # .encode() converts the char type into byte types
        InputSpeed1 = speed_entry1.get().encode()
        InputTheta2 = theta_entry2.get().encode()
        InputSpeed2 = speed_entry2.get().encode()
        #print (type(InputTheta))
        x = b'A,' + InputTheta1 + b',' + InputSpeed1 + b','+ InputTheta2 + b',' + InputSpeed2 + b','
        ser.write(x)

def stepperEnable():
    global stepper_enable_state
    if (stepper_enable_state):
        stepperEnable_button.configure(text = "Stepper Motors Disabled!", bg = "red")
        stepper_enable_state = False
        print(stepper_enable_state)
    else:
        stepperEnable_button.configure(text = "Stepper Motors Enabled!", bg = "green")
        stepper_enable_state = True
        print(stepper_enable_state)

def endeffectorEnable():
    global endeffector_enable_state
    if (endeffector_enable_state):
        endeffectorEnable_button.configure(text="End effector Disabled!", bg = "red")
        endeffector_enable_state = False
    else:
        endeffectorEnable_button.configure(text="End effector Enabled!", bg = "green")
        endeffector_enable_state = True    

    
def updatelabel():
    serialString = ser.readline() #reads the entire line of the serial port
    StringDecode = serialString.decode('Ascii') #converts the string of characters to Ascii
    StringDelimited = StringDecode.split(',') #splits the string with commas and returns a list of the encoder feedback positions
    #print (StringDelimited)

    if (len(StringDelimited) > 1): #This checks to see if the length of the string is greater than 1 (basically checks serial port for encoder feedback)
        # once the condition is met, it will store each element of the list and update the label text in the GUI
        EncoderTheta_1 = StringDelimited[0]
        EncoderTheta_2 = StringDelimited[1]
        label1=tk.Label(root, text = EncoderTheta_1)
        label1.grid(row=2,column=2)
        label2=tk.Label(root, text = EncoderTheta_2)
        label2.grid(row=6,column=2)

    root.after(1 ,updatelabel)
    #.after(delay(milliseconds),command) - root.after method is basically a loop and runs this function every _ milliseconds
    # based on many forums, youre supposed to use this loop to constantly update the GUI for the tkinter library rather than while/if loops
    
    

#-------------------------------------------------------------------- GUI LOOP ---------------------------------------------------------------
root = tk.Tk() #this starts and creates the GUI

#-------------------------------------------------------------- Size and Title of GUI --------------------------------------------------------
root.geometry("700x500")
root.title('Continuum Robot GUI')
# ------------------------------------------------------------- Buttons n shit ---------------------------------------------------------------
root.update()
run_button = tk.Button(root, text ="Run", command=run, font = 'sans 16 bold')
run_button.grid(row=0, column=0) 

stepperEnable_button = tk.Button(root, text = "Stepper Motors Disabled!", command = stepperEnable , bg = "red", font = 'sans 12 bold')
stepperEnable_button.grid(row=1, column=0)

endeffectorEnable_button = tk.Button(root, text = "End effector Disabled!", command = endeffectorEnable , bg = "red", font = 'sans 12 bold')
endeffectorEnable_button.grid(row=2,column=0)
#------------------------------------------------------------------ Stepper Motor 1 ----------------------------------------------------------
theta_label1 = tk.Label(root, text = "Position 1 (angle): " , bd = 2)
theta_label1.grid(row=0,column=1)

theta_entry1 = tk.Entry(root, bd=1)
theta_entry1.grid(row=0,column=2)

speed_label1 = tk.Label(root, text = "Speed 1 (steps/sec): " , bd = 2)
speed_label1.grid(row=1,column=1)

speed_entry1 = tk.Entry(root, bd=1)
speed_entry1.grid(row=1,column=2)

speednote_label1 = tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)" , bd = 2)
speednote_label1.grid(row=1,column=3)

encoder_label1 = tk.Label(root, text = "Encoder 1 Feedback: " , bd = 2)
encoder_label1.grid(row=2,column=1)

#------------------------------------------------------------------ Stepper Motor 2 ----------------------------------------------------------
space1 = tk.Label(root, text = "", bd = 2)
space1.grid(row=3,column=1)

theta_label2 = tk.Label(root, text = "Position 2 (angle): " , bd = 2)
theta_label2.grid(row=4,column=1)

theta_entry2 = tk.Entry(root, bd=1)
theta_entry2.grid(row=4,column=2)

speed_label2 = tk.Label(root, text = "Speed 2 (steps/sec): " , bd = 2)
speed_label2.grid(row=5,column=1)

speed_entry2 = tk.Entry(root, bd=1)
speed_entry2.grid(row=5,column=2)

speednote_label2 = tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)" , bd = 2)
speednote_label2.grid(row=5,column=3)

encoder_label2 = tk.Label(root, text = "Encoder 2 Feedback: " , bd = 2)
encoder_label2.grid(row=6,column=1)

#------------------------------------------------------------------ Stepper Motor 3 ----------------------------------------------------------
space2 = tk.Label(root, text = "", bd = 2)
space2.grid(row=7,column=1)

theta_label3 = tk.Label(root, text = "Position 3 (angle): " , bd = 2)
theta_label3.grid(row=8,column=1)

theta_entry3 = tk.Entry(root, bd=1)
theta_entry3.grid(row=8,column=2)

speed_label3 = tk.Label(root, text = "Speed 3 (steps/sec): " , bd = 2)
speed_label3.grid(row=9,column=1)

speed_entry3 = tk.Entry(root, bd=1)
speed_entry3.grid(row=9,column=2)

speednote_label3 = tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)" , bd = 2)
speednote_label3.grid(row=9,column=3)

encoder_label3 = tk.Label(root, text = "Encoder 3 Feedback: " , bd = 2)
encoder_label3.grid(row=10,column=1)

#------------------------------------------------------------------ Stepper Motor 4 ----------------------------------------------------------
space3 = tk.Label(root, text = "", bd = 2)
space3.grid(row=11,column=1)

theta_label4 = tk.Label(root, text = "Position 3 (angle): " , bd = 2)
theta_label4.grid(row=12,column=1)

theta_entry4 = tk.Entry(root, bd=1)
theta_entry4.grid(row=12,column=2)

speed_label4 = tk.Label(root, text = "Speed 3 (steps/sec): " , bd = 2)
speed_label4.grid(row=13,column=1)

speed_entry4 = tk.Entry(root, bd=1)
speed_entry4.grid(row=13,column=2)

speednote_label4 = tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)" , bd = 2)
speednote_label4.grid(row=13,column=3)

encoder_label4 = tk.Label(root, text = "Encoder 3 Feedback: " , bd = 2)
encoder_label4.grid(row=14,column=1)
#-----------------------------------------------------------------------------------------------------------------------------------------------
updatelabel()
root.mainloop() #This ends the GUI