from numpy import *
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QHBoxLayout,QVBoxLayout,QSlider,QLabel,QMessageBox,QListWidget,QTextEdit
# Create object serial port

### START QtApp #####
app = QtGui.QApplication([])            # you MUST do this once (initialize things)
####################
win = QWidget()


plot_uart = pg.GraphicsLayoutWidget() # creates a window
p = plot_uart.addPlot(title="Realtime plot")  # creates empty space for the plot in the window

ser = serial.Serial()
portName = "COM40" 
detached = False

main_layout = QVBoxLayout()
def clickMethod():
    print('Clicked Pyqt button.')
def connectCOM():
    try:                     # replace this port name by yours!
        baudrate = 115200
        global ser,portName,detached
        ser = serial.Serial(portName,baudrate)
        if detached:
            detached = False
        while not detached: 
            update()

    except Exception as e: 
        print(e)
        alert = QMessageBox()
        alert.setText('Error connect COM!')
        textEdit.setPlainText(str(e))
        alert.exec_()
def detachCOM():
    global ser,detached
    detached = True
    ser.close()
def list_clicked():
    global portName
    portName = list_widget.currentItem().text()
slider = QSlider()
slider.setValue(0)
slider.setMaximum(10000)

slider1 = QSlider()
slider1.setValue(0)
slider1.setMaximum(10000)

slider2 = QSlider()
slider2.setValue(0)
slider2.setMaximum(10000)

slider3 = QSlider()
slider3.setValue(0)
slider3.setMaximum(10000)

slider4 = QSlider()
slider4.setValue(0)
slider4.setMaximum(10000)

slider5 = QSlider()
slider5.setValue(0)
slider5.setMaximum(10000)

slider6 = QSlider()
slider6.setValue(0)
slider6.setMaximum(10000)

slider7 = QSlider()
slider7.setValue(0)
slider7.setMaximum(10000)

 # Will print '5'

layout = QHBoxLayout()
layout_inner = QHBoxLayout()
layout_inner1 = QHBoxLayout()
layout_inner2 = QVBoxLayout()

label = QLabel("Kp 1")
label1 = QLabel("Ki 1")
label2 = QLabel("Kd 1")

label3 = QLabel("Kp 2")
label4 = QLabel("Ki 2")
label5 = QLabel("Kd 2")

label6 = QLabel("Motor 1")
label7 = QLabel("Motor 2")

connect_button = QPushButton('Connect COM')
connect_button.clicked.connect(connectCOM)

detatch_button = QPushButton('Disconnect COM')
detatch_button.clicked.connect(detachCOM)

layout_inner2.addWidget(connect_button)
layout_inner2.addWidget(detatch_button)

list_widget = QListWidget()

list_widget.insertItem(0, "COM38" )
list_widget.insertItem(1, "COM39" )
list_widget.insertItem(2, "COM40" )
list_widget.insertItem(3, "COM41" )
list_widget.insertItem(4, "COM42" )
list_widget.insertItem(5, "COM43" )
list_widget.insertItem(6, "COM44" )
list_widget.insertItem(7, "COM45" )

list_widget.clicked.connect(list_clicked)
layout_inner.addWidget(list_widget)
layout_inner.addLayout(layout_inner2)

layout.addWidget(slider)
layout.addWidget(label)

layout.addWidget(slider1)
layout.addWidget(label1)

layout.addWidget(slider2)
layout.addWidget(label2)

layout.addWidget(slider3)
layout.addWidget(label3)

layout.addWidget(slider4)
layout.addWidget(label4)

layout.addWidget(slider5)
layout.addWidget(label5)

layout.addWidget(slider6)
layout.addWidget(label6)

layout.addWidget(slider7)
layout.addWidget(label7)

layout.addStretch(2)

layout.addLayout(layout_inner)

main_layout.addLayout(layout)

button = QPushButton('Set PID')
button.clicked.connect(clickMethod)

button1 = QPushButton('Start')
button1.clicked.connect(clickMethod)

button2 = QPushButton('Stop')
button2.clicked.connect(clickMethod)

layout_inner1.addWidget(button)
layout_inner1.addWidget(button1)
layout_inner1.addWidget(button2)

textEdit = QTextEdit()

main_layout.addLayout(layout_inner1)
main_layout.addWidget(plot_uart)
main_layout.addWidget(textEdit)

win.setLayout(main_layout)
win.show()
curve = p.plot()                        # create an empty "plot" (a curve to plot)

windowWidth = 500                       # width of the window displaying the curve
Xm = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr = -windowWidth                      # set first x position

# Realtime data plot. Each time this function is called, the data display is updated
def update():
    global curve, ptr, Xm, detached    
    Xm[:-1] = Xm[1:]                      # shift data in the temporal mean 1 sample left
    value = ser.readline()                # read line (single value) from the serial port
    try:
        Xm[-1] = float(value.decode().replace('\x00',''))                 # vector containing the instantaneous values 
    except Exception as e: 
        print(e)   
        textEdit.setPlainText(str(e))
        Xm[-1] = Xm[0]
    ptr += 1                              # update x position for displaying the curve
    curve.setData(Xm)                     # set the curve with this data
    curve.setPos(ptr,0)                   # set x position in the graph to 0
    QtGui.QApplication.processEvents()    # you MUST process the plot now
    label.setText("Kp 1 \n" + str(slider.value()/100))
    label1.setText("Ki 1 \n" + str(slider1.value()/100))
    label2.setText("Kd 1 \n" + str(slider2.value()/100))
    label3.setText("Kp 2 \n" + str(slider3.value()/100))
    label4.setText("Ki 2 \n" + str(slider4.value()/100))
    label5.setText("Kd 2 \n" + str(slider5.value()/100))
    label6.setText("Motor 1 \n" + str(slider6.value()))
    label7.setText("Motor 2 \n" + str(slider7.value()))

### MAIN PROGRAM #####    
# this is a brutal infinite loop calling your realtime data plot


### END QtApp ####

app.exec_() # you MUST put this at the end
##################