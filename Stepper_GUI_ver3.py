import serial
import tkinter as tk

comPort = "COM4"
ser = serial.Serial(comPort, baudrate = 115200, timeout = 0) #timeout is in seconds
serialString = "" #initilizes string for serial port

#----------------------------------------------------------------- FUNCTIONS ----------------------------------------------------------------

def run():
    InputTheta = theta_entry.get().encode() # .encode() converts the char type into byte types
    InputSpeed = speed_entry.get().encode()
    InputTheta2 = theta_entry2.get().encode()
    InputSpeed2 = speed_entry2.get().encode()
    #print (type(InputTheta))
    x = b'C,' + InputTheta + b',' + InputSpeed + b','+ InputTheta2 + b',' + InputSpeed2 + b','
    ser.write(x)

def updatelabel():
    serialString = ser.readline() #reads the entire line of the serial port
    StringDecode = serialString.decode('Ascii') #converts the string of characters to Ascii
    StringDelimited = StringDecode.split(',') #splits the string with commas and returns a list of the encoder feedback positions
    print (StringDelimited)

    if (len(StringDelimited) > 1): #This checks to see if the length of the string is greater than 1 (basically checks serial port for encoder feedback)
        # once the condition is met, it will store each element of the list and update the label text in the GUI
        EncoderTheta_1 = StringDelimited[0]
        EncoderTheta_2 = StringDelimited[1]
        label=tk.Label(root, text = EncoderTheta_1)
        label.grid(row=2,column=3)
        label2=tk.Label(root, text = EncoderTheta_2)
        label2.grid(row=5,column=3)

    root.after(16,updatelabel)
    #.after(delay(milliseconds),command) - root.after method is basically a loop and runs this function every _ milliseconds
    # based on many forums, youre supposed to use this loop to constantly update the GUI for the tkinter library rather than while/if loops
    
    

#-------------------------------------------------------------------- GUI LOOP ---------------------------------------------------------------
root = tk.Tk() #this starts and creates the GUI

#---------------------------------------------------- Size and Title of GUI -------------------------------------------------------------------
root.geometry("600x300")
root.title('Continuum Robot GUI')
#--------------------------------------------------------------------------- Stepper Motor 1 --------------------------------------------------
run_button = tk.Button(root, text ="Run", command=run)
run_button.grid(row=0, column=0) 

theta_label=tk.Label(root, text = "Position1 (angle): ")
theta_label.grid(row=0,column=2)

theta_entry=tk.Entry(root, bd=1,)
theta_entry.grid(row=0,column=3)

speednote_label=tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)")
speednote_label.grid(row=1,column=1)

speed_label=tk.Label(root, text = "Speed (steps/sec): ")
speed_label.grid(row=1,column=2)

speed_entry = tk.Entry(root, bd=1,)
speed_entry.grid(row=1,column=3)

encoder_label=tk.Label(root, text = "Encoder1 Feedback: ")
encoder_label.grid(row=2,column=2)


#--------------------------------------------------------------------------- Stepper Motor 2 --------------------------------------------------

theta_label2=tk.Label(root, text = "Position2 (angle): ")
theta_label2.grid(row=3,column=2)

theta_entry2 = tk.Entry(root, bd=1,)
theta_entry2.grid(row=3,column=3)

speednote_label2=tk.Label(root, text = "(Note: dont exceed 200 steps/sec!)")
speednote_label2.grid(row=4,column=1)

speed_label2=tk.Label(root, text = "Speed (steps/sec): ")
speed_label2.grid(row=4,column=2)

speed_entry2=tk.Entry(root, bd=1,)
speed_entry2.grid(row=4,column=3)

encoder_label2=tk.Label(root, text = "Encoder2 Feedback: ")
encoder_label2.grid(row=5,column=2)


#---------- Stepper Motor 3 ------------------------
#---------- Stepper Motor 4 ------------------------

updatelabel()
root.mainloop() #This ends the GUI