#! /usr/bin/env python3
#  -*- coding: utf-8 -*-
#
# Support module generated by PAGE version 8.0
#  in conjunction with Tcl version 8.6
#    Jun 20, 2024 10:41:18 AM BST  platform: Windows NT

import sys
import tkinter as tk
import tkinter.ttk as ttk
from tkinter.constants import *

import sys
import serial
import time
import random
import threading

import support_FreeBot

port = 'COM12'
baud_rate = 115200


_debug = True # False to eliminate debug printing from callback functions.

def main():
    '''Main entry point for the application.'''
    # global root
    root = tk.Tk()
    root.protocol( 'WM_DELETE_WINDOW' , root.destroy)
    # Creates a toplevel widget.
    global _top1, _w1
    _top1 = root
    _w1 = support_FreeBot.Toplevel1(_top1)
    root.mainloop()

global running

import sct

class Robot:

    def __init__(self, path):

        # Init controller
        self.sct = sct.SCT(path)

        ### Add callback functions for each event
        # Sensors (uncontrollable events)
        self.sct.add_callback(self.sct.EV['EV_btnMove'], None, self._check_btnMove, None)
        self.sct.add_callback(self.sct.EV['EV_btnStop'], None, self._check_btnStop, None)
        # Actions (controllable events)
        self.sct.add_callback(self.sct.EV['EV_move'], self._callback_move, None, None)
        self.sct.add_callback(self.sct.EV['EV_stop'], self._callback_stop, None, None)

    def control_step(self):
        self.sct.run_step()

    # Callback functions (sensors)
    def _check_btnMove(self, data):
        # print('Checking if btnMove was pressed...')
        if _w1.move_flag:
            print('EV: move pressed')
            return True
        else :
            # print('EV: move not pressed')
            return False

    def _check_btnStop(self, data):
        # print('Checking if btnStop was pressed...')
        #_check_btnStop=support_FreeBot.stop_flag
        if _w1.stop_flag:
            print('EV: stop pressed')
            return True
        else :
            # print('EV: stop not pressed')
            return False
    
    # Callback functions (actions)
    def _callback_move(self, data):
        print('Moving')
        global running 
        running = True
        move_thread = threading.Thread(target=self.move)
        move_thread.daemon = True
        move_thread.start()

        # data = '1'
        # self.send_data(port, baud_rate, data + '\n')

    def _callback_stop(self, data):
        print('Stopped')
        global running
        running = False
        data = '0'
        self.send_data(port, baud_rate, data + '\n')
    
    def move(self):
        global running
        while running:
            print('move in running')
            data = '1'
            self.send_data(port, baud_rate, data + '\n')
            delay=random.uniform(0.1,1.5)
            time.sleep(delay)
            if running==False:
                break
            
            data = 'B'
            self.send_data(port, baud_rate, data + '\n')
            delay=random.uniform(0.1,1.6)
            print(f"delay: ",delay)
            time.sleep(delay)
            if running==False:
                break
    
    def send_data(self, port, baud_rate, data):
        try: 
            print('trying to send data')
            # Open serial port
            ser = serial.Serial(
                port, 
                baud_rate, 
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
                )
            print(f"Opened port {port} with baud rate {baud_rate}")

            # Send data
            ser.write(data.encode())
            print(f"Sent data: {data}")
        
        except serial.SerialException as e:
            print(f"Error opening or using the serial port: {e}")


if __name__ == '__main__':
    robot = Robot('controller.yaml')
    print('loaded')

    thread = threading.Thread(target=main)
    thread.daemon = True  # background thread will exit if main thread exits
    thread.start()  # start tk loop

    while True:  # run in main thread
        time.sleep(0.1)

        robot.control_step()


    # loop_1=robot.control_step()
    # loop_2=main()
    # print ('threading has been declared')
    # thread1 = threading.Thread(target=loop_1)
    # thread2 = threading.Thread(target=loop_2)

    # print('threading has begun')

    # thread1.start()
    # thread2.start()


    
    # while True:
    #     robot.control_step()






# def start_move():
#     global running 
#     running = True
#     move_thread = threading.Thread(target=move)
#     move_thread.start()


# def move():
#     global running
#     while running:
#         data = '1'
#         send_data(port, baud_rate, data + '\n')
#         delay=random.randint(1,10)
#         time.sleep(delay)
        
#         data = 'B'
#         send_data(port, baud_rate, data + '\n')
#         delay=random.uniform(0.1,1.6)
#         print(f"delay: ",delay)
#         time.sleep(delay)
        
        

# def stop():
#     global running
#     running = False
#     data = '0'
#     send_data(port, baud_rate, data + '\n')

    



# def send_data(port, baud_rate, data):
#     try:
#         # Open serial port
#         ser = serial.Serial(
#             port, 
#             baud_rate, 
#             timeout=1,
#             parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE,
#             bytesize=serial.EIGHTBITS
#             )
#         print(f"Opened port {port} with baud rate {baud_rate}")

#         # Send data
#         ser.write(data.encode())
#         print(f"Sent data: {data}")

#         # Wait to ensure data is transmitted
#         time.sleep(1)
    
#     except serial.SerialException as e:
#         print(f"Error opening or using the serial port: {e}")




