#!/usr/bin/env python
# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.
# sudo systemctl start firmwared
# sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=::simple_front_cam=true
# source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
# python ./dronewebcam.py

import csv
import cv2
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
from anafiRequestPost import Anafi_Request_Post
from anafiScanning import Anafi_Scanning
from pyzbar import pyzbar
import time
import numpy as np

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingEvent import moveByEnd
from olympe.messages.ardrone3.Piloting import moveBy, CancelMoveBy
from olympe.messages.ardrone3.Piloting import moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AltitudeChanged, GpsLocationChanged, PositionChanged, moveByChanged, AltitudeAboveGroundChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.move import extended_move_by
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

# DRONE_IP = "192.168.42.1"
DRONE_IP = "10.202.0.1"

class AnafiConnection(threading.Thread):
 
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)

        # webcam 
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        # FPS = 1/X
        # X = desired FPS
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)

        # initialize the known distance from the camera to the object (referring to picture)
        self.KNOWN_DISTANCE = 25.1

        # initialize the known object width, which in this case, the barcode(referring to QR code in picture)
        self.KNOWN_WIDTH = 2.16535

        # change path to your own path
        self.image = cv2.imread("/home/dragonfly/UsingController/images/barcode.jpg")

        # decode the QR code in the picture
        self.foundBarcode = pyzbar.decode(self.image)

        # loop the found barcode and get the width (in pixels) for the QR code in the picture, then calculate the focal length
        for QRCode in self.foundBarcode:
            (x, y, w, h) = QRCode.rect
        self.focalLength = (w * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH

        self.request_post = Anafi_Request_Post()
        self.scanning_decode = Anafi_Scanning()

        # comment/uncomment this part if want to read location from txt file
        self.listOfLocation = self.request_post.readLocation()

        # flag for the current location and status
        self.currentLocation = None
        self.currentLocationStatus = False

        # initializing the barcode data list for storing the list of barcode that will be scanned later
        self.barcodeDataList = []
        
        # initializing for forward and backward movement to avoid crash
        self.fbRange = [70000, 90000]
        self.fb = 0
        # self.inst = ["UP", "DOWN", "LEFT", "RIGHT", "TAKEOFF"]
        self.barcodeData = ""
        self.moveup = 0
        self.i = 0
        self.inst = "NONE"
        self.rackInst = ["UP", "DOWN", "RIGHT", "LEFT"]
        self.inst1 = ""

        #for distance x and distance y for scanning area
        self.final_dX = 0.0
        self.final_dY = 0.0
        
        self.dY_1 = 100
        self.dX_2 = 100
        self.dY_2 = 100
        self.dX_3 = 100
        self.dY_3 = 100
        self.dX_4 = 100
        self.dY_4 = 100
        self.dX_5 = 100
        self.dY_5 = 100

        # self.dY_1 = 1
        # self.dX_2 = 0
        # self.dY_2 = 5
        # self.dX_3 = 5
        # self.dY_3 = 5
        # self.dX_4 = 0
        # self.dY_4 = 1
        # self.dX_5 = -5
        # self.dY_5 = 1
        self.droneOperation = 1
        self.itemlocation = ""
        super().__init__()
        super().start()
        self.start()
        # self.run()

    
    def start(self):
        # Connect the the drone
        self.drone.connect()
        print("drone connected")
        print("Battery percentage before takeoff:", self.drone.get_state(BatteryStateChanged)["percent"])
        print("Altitude before takeoff(should be 0):", self.drone.get_state(AltitudeChanged)["altitude"])
        # print(self.drone.get_state(PositionChanged))
        # self.run()

    # Properly stop the video stream and disconnect
    def stop(self):
        self.drone.disconnect()

    def show_yuv_frame(self, window_name, yuv_frame):
        # x = round(0.5)
        cv2frame = yuv_frame
        
        # scan the barcode, draw box and data in the frame
        self.barcodeInfo  = self.scanning_decode.startScanning(cv2frame, self.focalLength, self.KNOWN_WIDTH)

        
        # if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):

        #     if self.i == 0:
        #         print("enter scanning items operation")
                
        #         dXX = (self.dX_3 - self.dX_5)/2
        #         dYY = ((self.dY_2 + self.dY_3)/2) - ((self.dY_1 + self.dY_4)/2)
        #         dX = math.ceil((dXX)*100)/100
        #         dY = math.ceil((dYY)*100)/100
                
        #         self.final_dX = dX
        #         self.final_dY = dY
        #         print("dX: ",self.final_dX)
        #         print("dY: ", self.final_dY)

        #         x = self.final_dY/1.0
        #         upmovement = round(x)
        #         print("movement: ", upmovement)
        #         up = math.ceil((self.final_dY/upmovement)*100)/100

        #         self.scanningItemMovement(upmovement, up, self.final_dX)
 

        #     # else:
        #         # print("lebih satu dah")

        #     self.i = self.i + 1

        # if (self.final_dY == -10):
        #     print("end scanning")
        #     self.land()
        #     self.stop()
 
        # self.barcodeInfo[0] ---> barcode
        # self.barcodeInfo[1] ---> info for [center(x, y) , area]
        #to differentate the instructions' tags with items' tags
        if not self.barcodeInfo:
            pass

        else:
        #  str(self.barcodeInfo) != "None":
            # print("Area", self.barcodeInfo[1][1])
            # forward backward to avoid crash
            self.fb = self.avoidCrash(self.barcodeInfo[1])
            # print(self.fb)
            
            self.barcodeData1 = str(self.barcodeInfo[0])
            # print(self.barcodeData)

            # to read the tags only once at a time 
            if self.barcodeData1 != self.barcodeData:
                self.barcodeData = str(self.barcodeInfo[0])
                codeInfo= str(self.barcodeData).split()
                if len(codeInfo) > 1: 
                    self.storeData(codeInfo[0])
                    self.inst = codeInfo[1]
                    # self.automove(codeInfo[1], self.fb)
                else:
                    self.storeData(codeInfo[0])
  
        # Use OpenCV to show this frame
        cv2.imshow(window_name, cv2frame)
        cv2.waitKey(self.FPS_MS)  # please OpenCV for 1 ms...

    def scanningItemMovement(self, count, y, x):
        # upmovement, up, self.final_dX
        # count = upmovement
        count2 = count
        # count = count
        print("x is ", x)
        print("y is ", y)

        # while count > 0:

        if count2%2 == 0:
            countType = "Even Number"
            print(countType)

            while count > 0:
                # print(count)
                print("count is: ", count)
                self.cancelmove()
                self.move_Up2(y)

                if count%2 == 0:
                    self.cancelmove()
                    # print("masuk sini")
                    self.move_Right2(x)
                    # count = count-1
                    # print("tolak satu")

                elif count%2 == 1:
                    self.cancelmove()
                    self.move_Left2(x)
                    # count = count-1

                count = count-1

                # else:
                #     print("asal tak turun")

            else:
                print("done")
                self.final_dY = -10

        elif count2 %2 == 1:
            countType = "Odd Number"
            print(countType)

            while count > 0:
                print("count is: ", count)
                self.cancelmove()
                self.move_Up2(y)

                if count%2 == 0:
                    self.cancelmove()
                    # print("masuk sini")
                    self.move_Left2(x)
                    # count = count-1

                elif count%2 == 1:
                    self.cancelmove()
                    self.move_Right2(x)
                    # count = count-1
                
                count = count-1
                # else:
                    # print("asal tak turun")
                    
            else:
                self.final_dY = -10

            # else:
            #     print("errorrrr")
        
        # return
    
    def scanItem(self):

        if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):

            if self.i == 0:
                print("enter scanning items operation")
                
                dXX = (self.dX_3 - self.dX_5)/2
                dYY = ((self.dY_2 + self.dY_3)/2) - ((self.dY_1 + self.dY_4)/2)
                dX = math.ceil((dXX)*100)/100
                dY = math.ceil((dYY)*100)/100
                
                self.final_dX = dX
                self.final_dY = dY
                print("dX: ",self.final_dX)
                print("dY: ", self.final_dY)

                x = self.final_dY/1.0
                upmovement = round(x)
                print("movement: ", upmovement)
                up = math.ceil((self.final_dY/upmovement)*100)/100

                self.scanningItemMovement(upmovement, up, self.final_dX)
 

            # else:
                # print("lebih satu dah")

            self.i = self.i + 1

        if (self.final_dY == -10):
            print("end scanning")
            self.land()
            self.stop()

    def avoidCrash(self, info):
        area = info[1]

        if area > self.fbRange[0] and area > self.fbRange[1]:
            fb = 0
        elif area > self.fbRange[1]:
            fb = -0.2
        elif area < self.fbRange[0] and area != 0:
            fb = 0.2

        return fb

    def storeData(self, barcodeData):
        # condition to check the barcode that have been scanned is an item ID or location ID
        # because of the library keep scanning and decode the frame, we need to set a condition if there is no QR code in the frame, just pass
        # if not self.barcodeData:
        #     pass
        # elif
        # print("barcode: " + barcodeData)
        if (barcodeData in self.listOfLocation):
            self.currentLocation = barcodeData
            self.currentLocationStatus = True
            print("location status true detected")
            print(self.barcodeDataList)

        elif (barcodeData not in self.barcodeDataList) and (self.currentLocationStatus == True):
            self.barcodeDataList.append(barcodeData)
            item_dY = str(math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100)
            # item_dX = str(math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100)
            item_dX = str(self.drone.get_state(moveByEnd)["dY"])
            self.itemlocation = "(" + item_dX + "," + item_dY + ")"

            self.request_post.sendData(barcodeData, self.currentLocation, self.itemlocation)
            print("send data to sendData() in request post")


    def automove(self, movement, fb):
 
        if movement == "UP":
            print("MOVE UP")
            
            if self.moveup<1:
                # time.sleep(1)
                self.moveup +=1
                # self.cancelmove()
                self.dY_1 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
                # self.dY_1 = self.drone.get_state(AltitudeChanged)["altitude"]
                # print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])
                print("self.dY_1: ", self.dY_1)
                # print("Position (latitude, longitude, altitude) :", self.drone.get_state(PositionChanged))
                dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
                print("Altitude Above Ground: ", dY_aboveGround)
                print(self.drone.get_state(PositionChanged))
                # print("moveByEnd (d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed) :", self.drone.get_state(moveByEnd))
                
                #stopmove() is to make sure the drone stop for a while before move to the next direction.. this is to avoid too much inertia
                self.stopmove()
                self.move_Up(self.fb, 8)
                
            else:
                self.cancelmove()
                # self.land() # will change to start scanning operation
                
                # print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])
                self.dY_5 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
                print("self.dY_5: ", self.dY_5)
                self.dX_5 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
                print("dX_5 :", self.dX_5)
                # print("Position (latitude, longitude, altitude) :", self.drone.get_state(PositionChanged))
                dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
                print("Altitude Above Ground: ", dY_aboveGround)
                
                # self.move_Up(self.fb, 5)
                print("End scanning rack tags")
                # self.scanItem()
                # if (self.dX_2 != 0.0) and (self.dX_3 != 0.0) and (self.dX_4 != 0.0) and (self.dX_5 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0):
                #     print("all dx and dy has values")

            
        elif movement == "DOWN":
            print("MOVE DOWN")
            self.cancelmove()
            
            self.dY_3 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            self.dX_3 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            print("Altitude Above Ground: ", dY_aboveGround)
            print("dY_3: ", self.dY_3)
            print("dX_3: ", self.dX_3)
            print(self.drone.get_state(PositionChanged))

            self.stopmove()
            self.move_Down(self.fb, 8)

        elif movement == "RIGHT":
            print("MOVE RIGHT")
            self.cancelmove()

            self.dY_2 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            self.dX_2 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            print("Altitude Above Ground: ", dY_aboveGround)
            print("dY_2: ", self.dY_2)
            print("dX_2: ", self.dX_2)

            self.stopmove()
            self.move_Right(self.fb, 8)

        elif movement == "LEFT":
            print("LEFT")
            self.cancelmove()

            self.dY_4 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            self.dX_4 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            print("Altitude Above Ground: ", dY_aboveGround)
            print("dY_4: ", self.dY_4)
            print("dX_4:", self.dX_4)
          
            self.stopmove()
            self.move_Left(self.fb, 8)

        else:
            self.stopmove()
            print("No such direction")
            print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])

        return


    def run(self):
        window_name = "Olympe Streaming"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        # while self.drone.connect():
        while main_thread.is_alive():
            if self.cap.isOpened():
                _, img = self.cap.read()
            
            try:
                self.show_yuv_frame(window_name, img)
                # if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):
                #     # self.scanItem()
                #     print("hahaha")
                #     # self.takeoff()
                #     self.scanItem()
                
            except Exception:
                # We have to continue popping frame from the queue even if
                # we fail to show one frame
                traceback.print_exc()

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     anafi_connection.stop()
        # cv2.destroyWindow(window_name)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                anafi_connection.stop()
            
        if cv2.waitKey(1) & 0xFF == ord('l'):
            anafi_connection.land()

        if cv2.waitKey(1) & 0xFF == ord('t'):
            anafi_connection.takeoff()

        # cv2.destroyWindow(window_name)
      

    def takeoff(self):
        self.drone(
            TakeOff()
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()
        return
    
    
    def land(self):
        self.drone(Landing()).wait().success()
        print("---------------------------------------------------SUCCESSFUL LAND---------------------------------------------------------------------")

    def cancelmove(self):
        self.drone(CancelMoveBy()).wait(2).success()
        self.fb = 0
        print("---------------------------------------------------SUCCESSFUL CANCEL---------------------------------------------------------------------")

        return

    def stopmove(self):
        self.drone(extended_move_by(0, 0, 0, 0, 0, 0, 0)).wait(1).success()
        print("---------------------------------------------------SUCCESSFUL REST---------------------------------------------------------------------")
       
        return

    # def move_Forward(self, range):
    #     distance = float(range)
    #     assert self.drone(
    #     moveBy(distance, 0, 0, 0)
    #     >> FlyingStateChanged(state="hovering", _timeout=5)
    #     )
    #     print("---------------------------------------------SUCCESSFUL FORWARD---------------------------------------------------------------------")

    #     return
    
    # def move_Backward(self, range):
    #     distance = -float(range)
    #     assert self.drone(
    #     moveBy(distance, 0, 0, 0)
    #     >> FlyingStateChanged(state="hovering", _timeout=5)
    #     )
    #     print("---------------------------------------------SUCCESSFUL BACKWARD---------------------------------------------------------------------")

    #     return
    
    def move_Right(self, fb, range):
        distance = float(range)
        #extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))
        self.drone(extended_move_by(fb, distance, 0, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL RIGHT---------------------------------------------------------------------")
        return

    def move_Right2(self, range):
        distance = float(range)
        #extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))
        self.drone(
            extended_move_by(0, distance, 0, 0, 2, 2, 0)
        ).wait().success()

        print("---------------------------------------------------SUCCESSFUL RIGHT---------------------------------------------------------------------")
        return

    def move_Left2(self, range):
        distance = -float(range)
        self.drone(
            extended_move_by(0, distance, 0, 0, 2, 2, 0)
        ).wait().success()
        print("---------------------------------------------------SUCCESSFUL LEFT---------------------------------------------------------------------")
        return

    def move_Up2(self, range):
        distance = -float(range)
        self.drone(
            extended_move_by(0, 0, distance, 0, 2, 2, 0)
        ).wait().success()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        print("---------------------------------------------------SUCCESSFUL UP---------------------------------------------------------------------")
        return

    def move_Left(self, fb, range):
        distance = -float(range)
        self.drone(extended_move_by(fb, distance, 0, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL LEFT---------------------------------------------------------------------")
        return
    
    def move_Up(self, fb, range):
        distance = -float(range)
        self.drone(extended_move_by(fb, 0, distance, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        # print("---------------------------------------------------SUCCESSFUL UP---------------------------------------------------------------------")
        return

    def move_Down(self, fb, range):
        distance = float(range)
        self.drone(extended_move_by(fb, 0, distance, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )ss
        # print("---------------------------------------------------SUCCESSFUL DOWN---------------------------------------------------------------------")
        return
    
    def fullyAutonomous(self):

        takeoffStatus = self.drone.get_state(FlyingStateChanged)["state"]
        # if self.takeoff() == False:
        if takeoffStatus == FlyingStateChanged_State.landed :
            print(takeoffStatus)
            self.takeoff()

        # while self.drone.get_state(FlyingStateChanged)["state"] == FlyingStateChanged_State.hovering :
        while self.droneOperation == 1 :

            if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):
                self.scanItem()

            elif self.inst in self.rackInst :
                if self.inst1 != self.inst:
                    self.inst1 = self.inst
                    self.automove(self.inst, self.fb)
                    # inst1 = self.inst
                else:
                    pass

            else:
                pass
                # self.stopmove()
                # while self.drone.connect():
                # print("no movement")
                # time.wait(5)
                # print("No Movement")
           



if __name__ == "__main__":
    anafi_connection = AnafiConnection()
    anafi_connection.start()
    anafi_connection.fullyAutonomous()
    anafi_connection.stop()

       
        


