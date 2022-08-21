#SPI Kommunikations-Script 27092020
#V3.5

import sys
import os
import time
import numpy as np

#spezielle RPI libs
import spidev
import RPi.GPIO as GPIO

#Register Adressen und Statics
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
INT_PINCFG = 0x0F

ACCEL_CONFIG = 0x14 # User Bank 2
ACCEL_XOUT_H = 0x2D # User Bank 0
ACCEL_XOUT_L = 0x2E # User Bank 0
ACCEL_YOUT_H = 0x2F # User Bank 0
ACCEL_YOUT_L = 0x30 # User Bank 0
ACCEL_ZOUT_H = 0x31 # User Bank 0
ACCEL_ZOUT_L = 0x32 # User Bank 0

GYRO_CONFIG_1 = 0x01 # User Bank 2
GYRO_XOUT_H = 0x33 # User Bank 0
GYRO_XOUT_L = 0x34 # User Bank 0
GYRO_YOUT_H = 0x35 # User Bank 0
GYRO_YOUT_L = 0x36 # User Bank 0
GYRO_ZOUT_H = 0x37 # User Bank 0
GYRO_ZOUT_L = 0x38 # User Bank 0

RAW_DATA_0_RDY_INT = 0x1C # User Bank 0
INT_ENABLE_1 = 0x11 # User Bank 0

#I2C_MST_CTRL = 0x01
#I2C_MST_DELAY_CTRL = 0x02
#I2C_SLV0_ADDR = 0x03
#I2C_SLV0_REG = 0x04
#I2C_SLV0_DO = 0x06
#I2C_SLV0_CTRL = 0x05
#I2C_SLV0_CTRL_SET = 0x89
#I2C_ADRESS_MAG_R = 0x8C
#I2C_ADRESS_MAG_W = 0x0C
#EXT_SLV_SENS_DATA_00 = 0x3B

WIA_MAG = 0x01
HXL = 0x11
HXH = 0x12
HYL = 0x13
HYH = 0x14
HZL = 0x15
HZH = 0x16
CNTL2 = 0x31
CNTL3 = 0x32

SRST = 0x01
MODE_SINGLE = 0x01
MODE_10 = 0x02
MODE_20 = 0x04
MODE_50 = 0x06
MODE_100 = 0x08

REG_BANK_SEL = 0x7F
USER_BANK_0 = 0x00
USER_BANK_1 = 0x10
USER_BANK_2 = 0x20
USER_BANK_3 = 0x30
RESET_VALUE = 0xC1
CLEAR_SLEEP_BIT = 0x01
USER_CTRL = 0x03

bus = 0             #Bus-Lane
device = 0          #CS = 0
frequenz = 400000     #in Hz
Sequ_Länge = 2.000  #Dauer der Sequenz in Sekunden
mode = "BOTH"       #Betriebsmodus des Sensors

#Platzhalter für Pinnummern des Interrupts und des Aktivierungstasters
Button_pin = 0
Interrupt_pin = 0

#Pfade zum speichern der Daten auf dem System
structur_path = "/home/imu_ai"
sequence_path = structur_path + "/Sequenzen"
ai_path = structur_path + "/ai"

#Bus vorbereiten und SPI-Objekt erzeugen
spi = spidev.SpiDev()
spi.open(bus, device)
spi.lsbfirst = False
time.sleep(0.001)

#System vorbereiten
def Init():
    #GPIO Methode und Inputs setzen
    GPIO.setmode(GPIO.BCM)
    #GPIO.setup(Button_pin, GPIO.IN)
    #GPIO.setup(Interrupt_pin, GPIO.IN)

    #Check auf bestehende Ordnerstruktur und zählen der bereits bestehenden Sequenzen etc.
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print("File is started from directory:" + dir_path)
    if(os.path.isdir(structur_path)):
        print("Ordnerstruktur erkannt")
        if(dir_path == structur_path):
            print("Script liegt im richtigen Verzeichnis")
        else:
            print("Script liegt im falschen Verzeichnis --- home/imu_ai sollte verwendet werden")
            exit()
        if(os.path.isdir(sequence_path)):
            print("Sequenzen vorhanden:")
            i = 0
            for name in os.listdir(sequence_path + "/"):
                if (os.path.isfile(sequence_path + "/" + name)):
                    i = i + 1
            print(i)
        else:
            print("Sequenzenordner fehlt")
            try:
                os.mkdir(sequence_path)
            except OSError:
                print ("Erstellen von Verzeichnis: %s Fehlgeschlagen" % sequence_path)
                exit()
            else:
                print ("Ordner %s erfolgreich erstellt" % sequence_path)
        if(os.path.isdir(ai_path)):
            print("KIs vorhanden:")
            i = 0
            for name in os.listdir(ai_path + "/"):
                if (os.path.isfile(ai_path + "/" + name)):
                    i = i + 1
            print(i)
        else:
            try:
                os.mkdir(ai_path)
            except OSError:
                print ("Erstellen von Verzeichnis: %s Fehlgeschlagen" % ai_path)
                exit()
            else:
                print ("Ordner %s erfolgreich erstellt" % ai_path)
    else:
        try:
            os.mkdir(structur_path)
        except OSError:
            print ("Erstellen von Verzeichnis: %s Fehlgeschlagen" % structur_path)
            exit()
        else:
            print ("Ordner %s erfolgreich erstellt" % structur_path)
        try:
            os.mkdir(sequence_path)
        except OSError:
            print ("Erstellen von Verzeichnis: %s Fehlgeschlagen" % sequence_path)
            exit()
        else:
            print ("Ordner %s erfolgreich erstellt" % sequence_path)
        try:
            os.mkdir(ai_path)
        except OSError:
            print ("Erstellen von Verzeichnis: %s Fehlgeschlagen" % ai_path)
            exit()
        else:
            print ("Ordner %s erfolgreich erstellt" % ai_path)


#SPI Schreib-Funktion
def SPI_write(register, data):
    msg = [register, data]
    msg[0] = msg[0] & 0x7F
    spi.xfer(msg)

#SPI Lese-Funktion
def SPI_read(register):
    msg = [register, 0x00]
    msg[0] = msg[0] | 0x80
    buffer = spi.xfer(msg)
    return buffer[1]

#SPI Lese-Funktion für schnelles sequenzielles Lesen (parallel SPI)
def SPI_Fullread():
    msg = [ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L, 0x00]
    for i in range(0, len(msg)):
        msg[i] = msg[i] | 0x80
    buffer = spi.xfer(msg)
    return buffer

#Konfigurieren des MCU SPI Chips
def Sensor_Initialisierung():

    spi.max_speed_hz = frequenz
    time.sleep(1)
    SPI_write(PWR_MGMT_1, CLEAR_SLEEP_BIT)
    time.sleep(1)
    SPI_write(PWR_MGMT_1, 0x80)
    time.sleep(3)
    SPI_write(REG_BANK_SEL, USER_BANK_0)
    time.sleep(0.000022)

    device_ident = SPI_read(0x00)
    print("Mein Identifier ist: ")
    print(device_ident)
    if (device_ident != 234):
        print("Falscher Identifier. Kommunikation funktioniert nicht!")
        exit()


    #user_cntrl_reg = user_cntrl_reg | 0xEF
    SPI_write(PWR_MGMT_1, 0x09)
    time.sleep(0.000022)
    PwrMgmt1 = SPI_read(PWR_MGMT_1)
    print("####################")
    print("Power Managment Register 1: (should be 9)")
    print(PwrMgmt1)
    print("####################")
    #SPI_write(USER_CTRL, 0x10)
    SPI_write(USER_CTRL, 0x90) #DMP an
    time.sleep(0.000022)
    user_cntrl_reg = SPI_read(USER_CTRL)
    print("####################")
    print("User Control Register: (should be 144)")
    print(user_cntrl_reg)
    print("####################")
    SPI_write(INT_ENABLE_1, 0x01)
    time.sleep(0.000022)


    if(mode == "ACC"):
        SPI_write(PWR_MGMT_2, 0x07)             # Acc einschalten, Gyro ausschalten
    elif(mode == "GYRO"):
        SPI_write(PWR_MGMT_2, 0x38)             # Acc ausschalten, Gyro einschalten
    elif(mode == "BOTH"):
        SPI_write(PWR_MGMT_2, 0x00)            # Beide einschalten
    else:
        print("No Mode set")
        exit()
    time.sleep(0.000022)


    SPI_write(REG_BANK_SEL, USER_BANK_2)    # wechsel der User bank zum auslesen der Sensor config und Sensitivität
    time.sleep(0.000022)
   # SPI_write(GYRO_CONFIG_1, 0x01)          # Reset der Config
   # time.sleep(0.000022)
   # SPI_write(ACCEL_CONFIG, 0x01)           # Reset der Config
   # time.sleep(1)                           # Wartezeit

    GyroSen_raw = SPI_read(GYRO_CONFIG_1)
    AccSen_raw = SPI_read(ACCEL_CONFIG)

    SPI_write(GYRO_CONFIG_1, (GyroSen_raw | 0x00)) # ---00- = +-250 dps; ---01- = +-500 dps; ---10- = +-1000 dps; ---11- = +-2000 dps;
    time.sleep(0.000022)
    SPI_write(ACCEL_CONFIG, (AccSen_raw | 0x00)) # ---00- = +-2g; ---01- = +-4g; ---10- = +-8gs; ---11- = +-16g;
    time.sleep(0.000022)

    GyroSen_raw = SPI_read(GYRO_CONFIG_1)   # Config lesen
    #print("{0:{fill}8b}".format(GyroSen_raw, fill='0'))
    GyroSen_bit = "{0:{fill}8b}".format(GyroSen_raw, fill='0') # gelesenen Int in 8 Bit interpretation verwandeln
    AccSen_raw = SPI_read(ACCEL_CONFIG)     # Config lesen
    #print("{0:{fill}8b}".format(AccSen_raw, fill='0'))
    AccSen_bit = "{0:{fill}8b}".format(AccSen_raw, fill='0') # gelesenen Int in 8 Bit interpretation verwandeln

    print("-------------------------------------------------")
    if((GyroSen_bit[6] == str(0)) & (GyroSen_bit[5] == str(0))):
        GyroSen = 2*250 #-250 °/s bis 250 °/s
    elif((GyroSen_bit[6] == str(1)) & (GyroSen_bit[5] == str(0))):
        GyroSen = 2*500 #-500 °/s bis 500 °/s
    elif((GyroSen_bit[6] == str(0)) & (GyroSen_bit[5] == str(1))):
        GyroSen = 2*1000 #-1000 °/s bis 1000 °/s
    elif((GyroSen_bit[6] == str(1)) & (GyroSen_bit[5] == str(1))):
        GyroSen = 2*2000 #-2000 °/s bis 2000 °/s
    else:
        print("Sensitivity not set")
        exit()
    if((AccSen_bit[6] == str(0)) & (AccSen_bit[5] == str(0))):
        AccSen = 2*2 #-2g bis 2g
    elif((AccSen_bit[6] == str(1)) & (AccSen_bit[5] == str(0))):
        AccSen = 2*4 #-4g bis 4g
    elif((AccSen_bit[6] == str(0)) & (AccSen_bit[5] == str(1))):
        AccSen = 2*8 #-8g bis 8g
    elif((AccSen_bit[6] == str(1)) & (AccSen_bit[5] == str(1))):
        AccSen = 2*16 #-16g bis 16g
    else:
        print("Sensitivity not set")
        exit()
    print("Gyro Sensitivity:")
    print(GyroSen)
    print("Acc Sensitivity:")
    print(AccSen)
    print("-------------------------------------------------")
    SPI_write(REG_BANK_SEL, USER_BANK_0)
    time.sleep(0.000022)
    return [AccSen, GyroSen]


#Test von Register lesen
def Data_Acquisition(AccSen, GyroSen):
    starting_t = time.time()
    buffer = time.localtime()
    Sequenzname = (str(buffer.tm_year) + "{0:{fill}2}".format(buffer.tm_mon, fill=0) + "{0:{fill}2}".format(buffer.tm_mday, fill=0) + "{0:{fill}2}".format(buffer.tm_hour, fill=0) + "{0:{fill}2}".format(buffer.tm_min, fill=0) + "{0:{fill}2}".format(buffer.tm_sec, fill=0))
    duration = 0
    try:
        file = open(sequence_path + "/" + Sequenzname + ".txt","a")
    except OSError:
        print ("Fehlende Rechte um Sequenzen zu speichern")
        exit()
    file.write("####################################" + "\n")
    file.write("Modus:," + mode + "," + "Frequenz:," + str(frequenz) + "," + "\n")
    file.write("Acc-Range:," + str(AccSen/2) + "," + "Gyro-Range:," + str(GyroSen/2) + "," + "\n")
    file.write("Start:," + str(starting_t) + "," + "\n")
    file.write("####################################" + "\n")
    FullBuffer = []
    while(duration <= Sequ_Länge):
        #if(GPIO.input(Interrupt_pin) == 1):
        #    Interrupt_Flag = 1
        #if(Interrupt_Flag == 1):
        if(SPI_read(RAW_DATA_0_RDY_INT) == 1):
            print("ready\n")
        current_t = time.time()
        duration = current_t - starting_t
        FullBuffer.append([bytes(SPI_Fullread()), current_t]) # Komplette Register im Parallel Read Lesen und als Bytes speichern, da 2s Complement verwendet wird
        
        #AccX = int.from_bytes([FullBuffer[1], FullBuffer[2]], byteorder="big", signed=True) #Richtige Interpretation des High und Lowbytes im 2s Complement; Byteorder = Highbyte First ("big")
        #AccY = int.from_bytes([FullBuffer[3], FullBuffer[4]], byteorder="big", signed=True)
        #AccZ = int.from_bytes([FullBuffer[5], FullBuffer[6]], byteorder="big", signed=True)
        #GyroX = int.from_bytes([FullBuffer[7], FullBuffer[8]], byteorder="big", signed=True)
        #GyroY = int.from_bytes([FullBuffer[9], FullBuffer[10]], byteorder="big", signed=True)
        #GyroZ = int.from_bytes([FullBuffer[11], FullBuffer[12]], byteorder="big", signed=True)

        #AccX = AccX / (2**16 / AccSen) # Aussteuerung teilen durch Sensitivität
        #AccY = AccY / (2**16 / AccSen)
        #AccZ = AccZ / (2**16 / AccSen)

        #GyroX = GyroX / (2**16 / GyroSen) # Aussteuerung teilen durch Sensitivität
        #GyroY = GyroY / (2**16 / GyroSen)
        #GyroZ = GyroZ / (2**16 / GyroSen)

        #print("--------------------------------")
        #print("-------Accelerometer-------")
        #print(str(AccX) + "g")
        #print(str(AccY) + "g")
        #print(str(AccZ) + "g")
        #print("-------Gyroscope-------")
        #print(str(GyroX) + "°/s")
        #print(str(GyroY) + "°/s")
        #print(str(GyroZ) + "°/s")
        #print("--------------------------------")

        #Interrupt_Flag = 0
        #current_t = time.time()
        #duration = current_t - starting_t
        #print("duration since start in ms:")
        #print(duration)
    end_t = time.time()
    #print(FullBuffer)
    for item in FullBuffer:
        file.write(str(item[1] - starting_t) + "," + hex(item[0][1]) + "," + hex(item[0][2]) + "," + hex(item[0][3]) + "," + hex(item[0][4]) + "," + hex(item[0][5]) + "," + hex(item[0][6]) + "," + hex(item[0][7]) + "," + hex(item[0][8]) + "," + hex(item[0][9]) + "," + hex(item[0][10]) + "," + hex(item[0][11]) + "," + hex(item[0][12]) + "\n")
    duration = end_t - starting_t
    print(duration)
    file.write("Ende:," + str(end_t) + "," + "Dauer:," + str(duration))
    file.close()
    del FullBuffer

#main Function
def main():
    Init()
    Sensitivity = Sensor_Initialisierung()
    while(1):
        #if(GPIO.input(Button_pin) == 1):
        Data_Acquisition(Sensitivity[0], Sensitivity[1])
        print("done")
        time.sleep(1)

main()
        #if(Acquisition_mode == 0 & GPIO.input(Button_pin) == 1):
        #    while(GPIO.input(Button_pin)):
        #        Acquisition_mode = 1
        #if(Acquisition_mode == 1):
        #    Data_Acquisition()