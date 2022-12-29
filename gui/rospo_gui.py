#!/usr/bin/python

import serial, time, sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons
import serial.tools.list_ports
import tkinter.filedialog
import csv

if( len(sys.argv) < 2 ):
    serportlist = [comport.device for comport in serial.tools.list_ports.comports()]
    serport = serportlist[0]
    print("Serial port not specified. Defaulting to "+serport+".\n")
    if len(serportlist) > 1:
        print("If this doesn't work, try others: "+str(serportlist)+"\n")
else:
    serport = str(sys.argv[1])

running = True

#initialization and open the port

#possible timeout values:

#    1. None: wait forever, block call

#    2. 0: non-blocking mode, return immediately

#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial()
ser.port = serport
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.timeout = 0.25          #block read
#ser.timeout = 0             #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 3     #timeout for write
ser.open()



if ser.isOpen():

        outmsg = bytearray([0,0,0,0,0,0,0,0,0])

    #try:

        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                     #and discard all that is in buffer

        #write data

        #ser.write("AT+CSQ=?\x0D")

        #print("write data: AT+CSQ=?\x0D")

        #time.sleep(0.5)  #give the serial port sometime to receive the data

        #numOfLines = 0
        
        fig, ax = plt.subplots()

        #x = np.arange(0, 2, 1)
        line, = ax.plot([0, 2048],[127, 127] )
        fig.subplots_adjust(right=0.8)
        
        line2, = ax.plot([0, 2048],[127, 127] )

        ax.axis([0,4096,-255,255]);
        
        
            
        def pause(event):
            global running
            global bpause
            running = not running
            bpause.label.set_text("PAUSE" if running else "RUN")
            
        def settingsupdate(event):
            global outmsg
            outmsg = bytearray([ord('S'),ord('T'),int(cbchanneltoggle.get_status()[0]),int(cbchanneltoggle.get_status()[1]),int(cbchanneltoggle.get_status()[2]),0,0,0,0]);
            
        def trigupdate(event):
            global outmsg
            trdict = {'No Trigger':0,'Rising Edge':1,'Falling Edge':2}
            outmsg = bytearray([ord('T'),ord('R'),int(trdict[rtrig.value_selected]),int(strig.val),0,0,0,0,0]);
            
        def savedata(event):
            fn = tkinter.filedialog.asksaveasfilename(defaultextension=".csv")
            f = open(fn, 'w', newline='')
            if f is None: # Cancel
                return
            wr = csv.writer(f)
            wr.writerow(['time','ch1','ch2'])
            for w in range(len(line.get_xdata())):
                wr.writerow([line.get_xdata()[w],line.get_ydata()[w],line2.get_ydata()[w]])
            f.close()


        

        # To save the animation, use e.g.
        #
        # ani.save("movie.mp4")
        #
        # or
        #
        # writer = animation.FFMpegWriter(
        #     fps=15, metadata=dict(artist='Me'), bitrate=1800)
        # ani.save("movie.mp4", writer=writer)
        
        axpause = fig.add_axes([0.82, 0.12, 0.15, 0.075])
        bpause = Button(axpause, 'PAUSE')
        bpause.on_clicked(pause)
        
        axsave = fig.add_axes([0.82, 0.05, 0.15, 0.05])
        bsave = Button(axsave, 'save data')
        bsave.on_clicked(savedata)
        
        axtrig = fig.add_axes([0.82, 0.71, 0.05, 0.22])
        strig = Slider(
            ax=axtrig,
            label="CH1\nTrigger",
            valmin=1,
            valmax=254,
            valinit=128,
            valstep=1,
            orientation="vertical"
            )
        strig.on_changed(trigupdate)
        
        
        axoffset = fig.add_axes([0.92, 0.71, 0.05, 0.22])
        soffset = Slider(
            ax=axoffset,
            label="CH1\nOffset",
            valmin=-255,
            valmax=255,
            valinit=0,
            valstep=1,
            orientation="vertical"
            )
            
        triggermarker, = ax.plot([0],[strig.val+soffset.val],marker=9,color='blue')
            
        axoffset2 = fig.add_axes([0.92, 0.25, 0.05, 0.22])
        soffset2 = Slider(
            ax=axoffset2,
            label="CH2\nOffset",
            valmin=-255,
            valmax=255,
            valinit=-255,
            valstep=1,
            orientation="vertical",
            color='red'
            )
            
        axchanneltoggle = fig.add_axes([0.82, 0.25, 0.1, 0.22])
        cbchanneltoggle = CheckButtons(axchanneltoggle, ['+/-','2chan','hi speed'])
        cbchanneltoggle.on_clicked(settingsupdate)
            
        def animate(i):
            global outmsg
            if( 0 != outmsg[0] ):
                ser.write(outmsg)
                outmsg = bytearray([0,0,0,0,0,0,0,0,0])
            runmsg = bytearray([ord('R'),ord('N'),0,0,0,0,0,0,0])
            ser.write(runmsg)
            ser.flushOutput()
            triggermarker.set_ydata(strig.val+soffset.val)
            triggermarker.set_xdata(ax.get_xlim()[0])
            #incomingbytes = ser.read_until("DATA"); # find start of data
            incomingbytes = ser.read()
            while b'D' != incomingbytes:
                #print(incomingbytes)
                incomingbytes = ser.read()
            if( b"ATA" == ser.read(3) ):
                #print(incomingbytes)
                #if incomingbytes == "DATA":
                numchannels = int.from_bytes(ser.read(),'little')
                #print(numchannels)
                #print("numchannels = ")
                #print(numchannels)
                incomingbytes = ser.read()  # 1 = toffset, 0 = simultaneous
                #print(int.from_bytes(incomingbytes,'little'))
                if 0 != int.from_bytes(incomingbytes,'little'):
                    toffset = True # interleaved samples at alternating sample times
                else:
                    toffset = False # simultaneous samples, transmitted interleaved
                #print( range(1,numchannels,1))
                #for chan in np.arange(0,numchannels,1):
                #print("updating...\n")
                incomingbytes = ser.read(2)
                #print(incomingbytes)
                lendata = int.from_bytes(incomingbytes,'little')
                if(0 == lendata or 1024*8 < lendata):
                    return line,line2,triggermarker
                #print(f"lendata = {lendata}\n")
                incomingbytes = ser.read(lendata)
                if lendata != len(incomingbytes):
                    print("Did not get a full frame: ",len(incomingbytes)," vs ",lendata)
                    return line,line2,triggermarker
                if running:
                    if( 1 == numchannels ):
                        plt.setp(line2, linestyle='None')
                        numbers = list(map(lambda x : x + soffset.val, incomingbytes))
                        if(len(numbers) != len(line.get_xdata())):
                            line.set_data(np.arange(0,len(numbers),1),numbers)
                        else:
                            line.set_ydata(numbers)  # update the data.
                    else: # both channels
                        plt.setp(line2, linestyle='-')
                        if(len(incomingbytes)/2 != len(line.get_xdata())):
                            numbers = list(map(lambda x : x + soffset.val, incomingbytes))
                            line.set_data(np.arange(0,len(numbers),2),numbers[0::2])
                            numbers = list(map(lambda x : x + soffset2.val, incomingbytes))
                            line2.set_data(np.arange(1,len(numbers),2),numbers[1::2])
                        else:
                            numbers = list(map(lambda x : x + soffset.val, incomingbytes))
                            line.set_ydata(numbers[0::2])  # update the data.
                            numbers = list(map(lambda x : x + soffset2.val, incomingbytes))
                            line2.set_ydata(numbers[1::2])  # update the data.
            
            return line,line2,triggermarker
            
        
        ani = animation.FuncAnimation( fig, animate, interval=20, blit=True, save_count=5)
        
        rax = fig.add_axes([0.81, 0.55, 0.18, 0.12])
        rtrig = RadioButtons(rax, ('No Trigger','Rising Edge', 'Falling Edge'))
        rtrig.on_clicked(trigupdate)

        plt.show()

        ser.close()

    #except Exception as e1:

        #print ("error: " + str(e1))

else:

    print ("cannot open serial port ")
 