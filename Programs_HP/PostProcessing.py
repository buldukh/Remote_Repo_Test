# 27200920
# V2.2 PostProces

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

Source_Path = "C:\Users\huso9\OneDrive - hs-pforzheim.de\01_Studying Master\2. Semester\Forschungsprojekt\01_Code\ai_imu-Entwicklung_Kommunikation\ai_imu-Entwicklung_Kommunikation/Sequenzen"
Target_Path = "C:\Users\huso9\OneDrive - hs-pforzheim.de\01_Studying Master\2. Semester\Forschungsprojekt\01_Code\ai_imu-Entwicklung_Kommunikation\ai_imu-Entwicklung_Kommunikation/Target"
Data_Path = Target_Path + "/Data"
Picture_Path = Target_Path + "/Pictures"

def PostProcess():
    sources = os.listdir(Source_Path + "/")
    i = 0
    if(os.path.isdir(Target_Path) == False):
        os.mkdir(Target_Path)
    if(os.path.isdir(Data_Path) == False):
        os.mkdir(Data_Path)
    if(os.path.isdir(Picture_Path) == False):
        os.mkdir(Picture_Path)
    for sequ in sources:
        if(os.path.isfile(Source_Path + "/" + sequ)):
            csvBuffer = pd.read_csv(Source_Path + "/" + sequ, skiprows=[0,1,2,3,4], skipfooter=1, engine='python').values
            PlotBuffer = [[],[],[],[],[],[]]
            FSource = open(Source_Path + "/" + sequ, "r")
            FirstLine = FSource.readline()
            SecondLine = FSource.readline()
            ThirdLine = FSource.readline()
            FourthLine = FSource.readline()
            FifthLine = FSource.readline()
            FSource.close()
            print("File:" + sequ)
            NumBuffer = ThirdLine.split(",")
            AccRange = float(NumBuffer[1]) * 2
            GyroRange = float(NumBuffer[3]) * 2
            AccSens = (2**16 / AccRange)
            GyroSens = (2**16 / GyroRange)
            print("Acc Sensitivity: " + str(AccSens))
            print("Gyro Sensitivity: " + str(GyroSens))
            try:
                file = open(Data_Path + "/" + sequ,"a")
            except:
                print("missing rights to make file")
                exit()
            for value in csvBuffer:
                file.write(str(value[0]) + ",")
                #byte_values = bytes([value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8], value[9], value[10] ,value[11], value[12]])
                #print(byte_values)
                #file.write(str(int.from_bytes(byte_values), byteorder="big", signed=True)) + ",")
                
                AccX = int.from_bytes(bytes([int(value[1], 16), int(value[2], 16)]), byteorder="big", signed=True)/AccSens
                AccY = int.from_bytes(bytes([int(value[3], 16), int(value[4], 16)]), byteorder="big", signed=True)/AccSens
                AccZ = int.from_bytes(bytes([int(value[5], 16), int(value[6], 16)]), byteorder="big", signed=True)/AccSens
                GyroX = int.from_bytes(bytes([int(value[7], 16), int(value[8], 16)]), byteorder="big", signed=True)/GyroSens
                GyroY = int.from_bytes(bytes([int(value[9], 16), int(value[10], 16)]), byteorder="big", signed=True)/GyroSens
                GyroZ = int.from_bytes(bytes([int(value[11], 16), int(value[12], 16)]), byteorder="big", signed=True)/GyroSens
                PlotBuffer[0].append(AccX)
                PlotBuffer[1].append(AccY)
                PlotBuffer[2].append(AccZ)
                PlotBuffer[3].append(GyroX)
                PlotBuffer[4].append(GyroY)
                PlotBuffer[5].append(GyroZ)
                file.write(str(AccX) + ",")
                file.write(str(AccY) + ",")
                file.write(str(AccZ) + ",")
                file.write(str(GyroX) + ",")
                file.write(str(GyroY) + ",")
                file.write(str(GyroZ) + "\n")
                #file.write(str(int.from_bytes([bytes(int(value[5], 16)), bytes(int(value[6], 16))], byteorder="big", signed=True)) + ",")
                #file.write(str(int.from_bytes([bytes(int(value[7], 16)), bytes(int(value[8], 16))], byteorder="big", signed=True)) + ",")
                #file.write(str(int.from_bytes([bytes(int(value[9], 16)), bytes(int(value[10], 16))], byteorder="big", signed=True)) + ",")
                #file.write(str(int.from_bytes([bytes(int(value[11], 16)), bytes(int(value[12], 16))], byteorder="big", signed=True)) + "\n")
            file.close()
            plt.plot(PlotBuffer[0], color='red', marker='None', linestyle='-')
            plt.plot(PlotBuffer[1], color='blue', marker='None', linestyle='-')
            plt.plot(PlotBuffer[2], color='green', marker='None', linestyle='-')
            plt.plot(PlotBuffer[3], color='red', marker='None', linestyle=':')
            plt.plot(PlotBuffer[4], color='blue', marker='None', linestyle=':')
            plt.plot(PlotBuffer[5], color='green', marker='None', linestyle=':')
            plt.savefig(Picture_Path + "/" + (sequ.split(".")[0] + ".jpg"))
            plt.close()
            del PlotBuffer
PostProcess()