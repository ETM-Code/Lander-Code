import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import openpyxl
from excelFunctions import appendExcel
from readValues import graphAfter
import math
import pickle
import struct

stop_event = threading.Event()

#TIMES AND MODES
realtime, mode = [0],[0]
launchTime, descendingTime, landedTime, avgTransmissionTime = 27.5439, 30.1589, 36.2895, 0.23
modeText = ['N/A']

#ACCELERATIONS
accelX, accelY, accelZ, accelVertical = [0],[0],[0],[0]
aXMax, aXMin, aYMax, aYMin, aZMax, aZMin, maxAccelVert, minAccelVert = 0, 0, 0, 0, 0, 0, 0, 0

#ANGLES
Xtilt, Ztilt, absoluteTilt = [0],[0],[0]
XtiltMax, ZtiltMax, absoluteTiltMax = 0, 0, 0
XtiltMin, ZtiltMin, absoluteTiltMin = 10000000000, 10000000000, 10000000000

#ENVIRONMENTAL SENSORS
pressure, temperature = [0],[0]
maxPressure, minPressure, refPress, maxTemperature, minTemp, refTemp = 0, 0, 0, 0, 0, 0

# ALTITUDES
altitudeE, maxAltE, minAltE, altitudeA, maxAltA, minAltA = [0], 0, 0, [0], 0, 0

# VELOCITIES
velX, vXMax, vXMin, velY, vYMax, vYMin, velZ, vZMax, vZMin, maxVelVert, minVelVert, velVertical = [0], 0, 0, [0], 0, 0, [0], 0, 0, 0, 0, [0]

# FORCES
forceX, fXMax, fXMin, forceY, fYMax, fYMin, forceZ, fZMax, fZMin, maxForceVert, minForceVert, forceVertical = [0], 0, 0, [0], 0, 0, [0], 0, 0, 0, 0, [0]

# CALIBRATIONS
slopePosX, calib0X, slopeNegX, slopePosY, calib0Y, slopeNegY, slopePosZ, calib0Z, slopeNegZ, calibTemp = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0


#MISCELANEOUS
serial_port = '/dev/cu.usbserial-0001'
baud_rate = 9600
timeout = 2
beginCalcs = 0
rocketMass = 0.1
capsuleMass = 0.05
g = 9.81
seaLevelPressure = 1013.25 #Sea Level Air Pressure in hPa

lists_to_process = [mode, modeText, accelX, accelY, accelZ, pressure, temperature, 
    altitudeE, altitudeA, velX, velY, velZ, forceX, forceY, forceZ, Xtilt, Ztilt, absoluteTilt]

def appendMaxMin(list, max, min):
    global aXMax, aXMin, aYMax, aYMin, aZMax, aZMin, maxAccelVert, minAccelVert, XtiltMax, ZtiltMax, absoluteTiltMax, XtiltMin, ZtiltMin, absoluteTiltMin, maxPressure, minPressure, refPress, maxTemperature, minTemp, refTemp, altitudeE, maxAltE, minAltE, altitudeA, maxAltA, minAltA, velX, vXMax, vXMin, velY, vYMax, vYMin, velZ, vZMax, vZMin, maxVelVert, minVelVert, velVertical, forceX, fXMax, fXMin, forceY, fYMax, fYMin, forceZ, fZMax, fZMin, maxForceVert, minForceVert, forceVertical
    if list[-1]>max:
        max = list[-1]

    if list[-1]<min:
        min = list[-1]

def equalize_list_lengths():
    global lists_to_process, mode, modeText, accelX, accelY, accelZ, pressure, temperature, altitudeE, altitudeA, velX, velY, velZ, forceX, forceY, forceZ, accelVertical, velVertical, forceVertical, Xtilt, Ztilt, absoluteTilt
    # Check each list in list_of_lists
    for lst in lists_to_process:
        while len(lst) < len(realtime):
            # Append the last value of lst if it has fewer elements than realtime
            lst.append(lst[-1])

        while len(realtime) < len(lst):
            # Append the last value of realtime if it has fewer elements than lst
            realtime.append(realtime[-1])

    
    return lists_to_process, realtime

def checkTransmissionTime():
    global avgTransmissionTime
    avgTransmissionTime = realtime[-1]/(len(realtime))

def equalize_before_plot(y_list, listName):
    #print(listName, " before equalization:  ", y_list[-1])
    if len(y_list) < len(realtime):
        print(listName, ":  ",len(y_list), "Realtime:  ", len(realtime))
        y_list += [y_list[-1]] * (len(realtime) - len(y_list))  # Append last value if shorter
        print(listName,":  ",len(y_list), "Realtime:  ", len(realtime))
    elif len(y_list) > len(realtime):
        print("Ah, length error (list longer), how quaint")
        y_list = y_list[:len(realtime)]  # Trim if longer
    #print(listName, " after equalization:  ", y_list[-1])
    return y_list

def calculateTilt():
    global Xtilt, Ztilt, absoluteTilt
    if accelX:  # Check if accelX is not empty
        # Ensure the value passed to math.asin is within [-1, 1]
        value = accelX[-1] / g
        clamped_value = max(min(value, 1), -1)
        tilt = math.asin(clamped_value)
        Xtilt.append(tilt)
        xComponent = math.sin(Xtilt[-1])
    else:
        Xtilt.append(0)
    if accelZ:  # Check if accelX is not empty
        # Ensure the value passed to math.asin is within [-1, 1]
        value = accelZ[-1] / g
        clamped_value = max(min(value, 1), -1)
        tilt = math.asin(clamped_value)
        Ztilt.append(tilt)
        zComponent = math.sin(Ztilt[-1])
    else:
        Ztilt.append(0)

    appendMaxMin(Xtilt,XtiltMax,XtiltMin)
    appendMaxMin(Ztilt,ZtiltMax,ZtiltMin)

    yComponent = math.cos(Xtilt[-1]) * math.cos(Ztilt[-1])
    vectorMagnitude = math.sqrt(xComponent**2+yComponent**2+zComponent**2)

    absoluteTilt.append(math.acos(yComponent/vectorMagnitude))
    appendMaxMin(absoluteTilt,absoluteTiltMax, absoluteTiltMin)

def calculateVerticalAcceleration():
    global accelVertical
    accelVertical.append(accelY[-1]*math.cos(absoluteTilt[-1]))

def calculateMaxAcceleration():
    global maxAccel, aXMax, aYMax, aZMax, maxAccelVert #Defining global variables
    appendMaxMin(accelX, aXMax, aXMin)
    appendMaxMin(accelY, aYMax, aYMin)
    appendMaxMin(accelZ, aZMax, aZMin)
    appendMaxMin(accelVertical, maxAccelVert, minAccelVert)

def calculateVelocity(aX, aY, aZ):
    global aXMax, aXMin, aYMax, aYMin, aZMax, aZMin, maxAccelVert, minAccelVert, XtiltMax, ZtiltMax, absoluteTiltMax, XtiltMin, ZtiltMin, absoluteTiltMin, maxPressure, minPressure, refPress, maxTemperature, minTemp, refTemp, altitudeE, maxAltE, minAltE, altitudeA, maxAltA, minAltA, velX, vXMax, vXMin, velY, vYMax, vYMin, velZ, vZMax, vZMin, maxVelVert, minVelVert, velVertical, forceX, fXMax, fXMin, forceY, fYMax, fYMin, forceZ, fZMax, fZMin, maxForceVert, minForceVert, forceVertical
    if abs(aX)>0.3: #Ensures that the acceleration taking place isn't noise
        velX.append(velX[-1]+(aX)*(realtime[-1]-realtime[-2])) #Steps forward velocity
    else:
        velX.append(velX[-1])
    if abs(aY)>0.3:
        velY.append(velY[-1]+(aY)*(realtime[-1]-realtime[-2]))
    else:
        velY.append(velY[-1])
    if abs(aZ)>0.3:
        velZ.append(velZ[-1]+(aZ)*(realtime[-1]-realtime[-2]))
    else:
        velZ.append(velY[-1])
    if abs(accelVertical[-1])>0.3:
        velVertical.append(velVertical[-1]+(accelVertical[-1])*(realtime[-1]-realtime[-2]))
    else:
        velVertical.append(velVertical[-1])

    appendMaxMin(velX, vXMax, vXMin)
    appendMaxMin(velY, vYMax, vYMin)
    appendMaxMin(velZ, vZMax, vZMin)
    appendMaxMin(velVertical, maxVelVert, minVelVert)

def calculateForce(aX, aY, aZ):
    global aXMax, aXMin, aYMax, aYMin, aZMax, aZMin, maxAccelVert, minAccelVert, XtiltMax, ZtiltMax, absoluteTiltMax, XtiltMin, ZtiltMin, absoluteTiltMin, maxPressure, minPressure, refPress, maxTemperature, minTemp, refTemp, altitudeE, maxAltE, minAltE, altitudeA, maxAltA, minAltA, velX, vXMax, vXMin, velY, vYMax, vYMin, velZ, vZMax, vZMin, maxVelVert, minVelVert, velVertical, forceX, fXMax, fXMin, forceY, fYMax, fYMin, forceZ, fZMax, fZMin, maxForceVert, minForceVert, forceVertical

    if mode[-1] == 1 or mode[-1] == 2:
        forceX.append(aX*rocketMass)
        forceY.append((aY+g)*rocketMass)
        forceZ.append(aZ*rocketMass)
        forceVertical.append(accelVertical[-1]*rocketMass)

    elif mode[-1] == 3 or mode[-1] ==4:
        forceX.append(aX*capsuleMass)
        forceY.append(aY*capsuleMass)
        forceZ.append(aZ*capsuleMass)
        forceVertical.append(accelVertical[-1]*rocketMass)
    else:
        print("Force Calculation Mode Error")
    
    appendMaxMin(forceX, fXMax, fXMin)
    appendMaxMin(forceY, fYMax, fYMin)
    appendMaxMin(forceZ, fZMax, fZMin)
    appendMaxMin(forceVertical, maxForceVert, minForceVert)

def calculateAverage(value):
    return sum(value)/len(value) if value else 0

def checkEnvironmentalMaxs():
    global pressure, maxPressure, temperature, maxTemperature
    appendMaxMin(pressure, maxPressure, minPressure)
    appendMaxMin(temperature, maxTemperature, minTemp)

def calculateEnvironmentalAltitude(press, temp):
    global altitudeE, maxAltE
    altitudeE.append(((press / refPress) ** (1 / -5.255877) - 1) * ((temp + 273.15) / 0.0065))
    appendMaxMin(altitudeE, maxAltE, minAltE)

def calculateAccelerometerAltitude(): #Attempts to calculate altitude from accelerometer values, for experimentation purposes
    global altitudeA, maxAltA
    altitudeA.append(altitudeA[-1]+velVertical[-1]*(realtime[-1]-realtime[-2]))
    if altitudeA[-1]<0:
        altitudeA[-1]=0
    appendMaxMin(altitudeA, maxAltA, minAltA)


def checkModeName(): #Maps the values sent by the lander to English mode names
    global launchTime, descendingTime, landedTime
    if checkForModeChange() == True:
        if mode[-1]==0:
            modeText.append('Error')
        elif mode[-1]==1:
            modeText.append('Pre-Launch')
        elif mode[-1]==2:
            modeText.append('Ascending')
            launchTime=realtime[-1]

        elif mode[-1]==3:
            modeText.append('Descending')  
            descendingTime=realtime[-1]

        else:
            modeText.append('Landed')
            landedTime=realtime[-1]
    elif launchTime != 0 and realtime[-1] - launchTime > 20:
        mode.append(4)
        modeText.append('Landed')

def checkForModeChange():
    if mode[-1]!=mode[-2]:
        return True
    else:
        return False
    

                                
def checkMode():
    if mode[-1] == 2 or mode[-1] == 3:
        if velVertical[-1] > 0:
            mode[-1] = 2
        else:
            mode[-1] = 3

def appendExcelFunc(): #Appends raw data to the excel sheet
    appendExcel(realtime, modeText, accelX, accelY, accelZ, accelVertical, velX, velY, velZ, velVertical, forceX, forceY, forceZ, forceVertical, altitudeE, altitudeA, pressure, temperature, maxAccelVert, aYMax, aXMax, aZMax, maxVelVert, vXMax, vYMax, vZMax, maxForceVert, fYMax, fXMax, fZMax, maxAltE, maxAltA, maxPressure, maxTemperature, launchTime, descendingTime, landedTime, avgTransmissionTime, Xtilt, Ztilt, absoluteTilt, minAccelVert, minForceVert, minVelVert, minAltA, minAltE, aXMin, aYMin, aZMin, vXMin, vYMin, vZMin, fXMin, fYMin, fZMin, XtiltMax, XtiltMin, ZtiltMax, ZtiltMin)

def map_accel_x(raw_x):
    if raw_x >= calib0X:
        # If the rawValue is between 0 and g, use the slope for that segment
        real_acceleration = slopePosX * (raw_x - calib0X)
    else:
        # If the rawValue is between -g and 0, use the slope for that segment
        real_acceleration = slopeNegX * (raw_x - calib0X)

    return real_acceleration

def map_accel_y(raw_y):
    if raw_y >= calib0Y:
        # If the rawValue is between 0 and g, use the slope for that segment
        real_acceleration = slopePosY * (raw_y - calib0Y)
    else:
        # If the rawValue is between -g and 0, use the slope for that segment
        real_acceleration = slopeNegY * (raw_y - calib0Y)

    return real_acceleration

def map_accel_z(raw_z):
    if raw_z >= calib0Z:
        # If the rawValue is between 0 and g, use the slope for that segment
        real_acceleration = slopePosZ * (raw_z - calib0Z)
    else:
        # If the rawValue is between -g and 0, use the slope for that segment
        real_acceleration = slopeNegZ * (raw_z - calib0Z)

    return real_acceleration


def saveLists():
    dataToPickle = {
        "realtime": realtime,
        "modeText": modeText,
        "accelX": accelX,
        "accelY": accelY,
        "accelZ": accelZ,
        "accelVertical": accelVertical,
        "velX": velX,
        "velY": velY,
        "velZ": velZ,
        "velVertical": velVertical,
        "forceX": forceX,
        "forceY": forceY,
        "forceZ": forceZ,
        "forceVertical": forceVertical,
        "altitudeE": altitudeE,
        "altitudeA": altitudeA,
        "pressure": pressure,
        "temperature": temperature,
        "maxAccelVert": maxAccelVert,
        "aYMax": aYMax,
        "aXMax": aXMax,
        "aZMax": aZMax,
        "maxVelVert": maxVelVert,
        "vXMax": vXMax,
        "vYMax": vYMax,
        "vZMax": vZMax,
        "maxForceVert": maxForceVert,
        "fYMax": fYMax,
        "fXMax": fXMax,
        "fZMax": fZMax,
        "maxAltE": maxAltE,
        "maxAltA": maxAltA,
        "maxPressure": maxPressure,
        "maxTemperature": maxTemperature,
        "launchTime": launchTime,
        "descendingTime": descendingTime,
        "landedTime": landedTime,
        "avgTransmissionTime": avgTransmissionTime,
        "Xtilt": Xtilt,
        "Ztilt": Ztilt,
        "absoluteTilt": absoluteTilt,
        "minAccelVert": minAccelVert,
        "minForceVert": minForceVert,
        "minVelVert": minVelVert,
        "minAltA": minAltA,
        "minAltE": minAltE,
        "aXMin": aXMin,
        "aYMin": aYMin,
        "aZMin": aZMin,
        "vXMin": vXMin,
        "vYMin": vYMin,
        "vZMin": vZMin,
        "fXMin": fXMin,
        "fYMin": fYMin,
        "fZMin": fZMin,
        "XtiltMax": XtiltMax,
        "XtiltMin": XtiltMin,
        "ZtiltMax": ZtiltMax,
        "ZtiltMin": ZtiltMin
    }
    with open('graphingLists.pkl', 'wb') as file:
        pickle.dump(dataToPickle, file)

broadcastedFinalResults = False


def averageDifference(lst):
    if len(lst) < 2:  # If the list has fewer than two elements, return 0 as there are no differences to calculate.
        return 0
    differences = [abs(lst[i] - lst[i-1]) for i in range(1, len(lst))]
    average_diff = sum(differences) / len(differences)
    return average_diff


def printData():
    print("Max Acceleration X:  ", max(accelX))
    print("Max Acceleration Y:  ", max(accelY))
    print("Max Acceleration Z:  ", max(accelZ))

    print("Min Acceleration X:  ", min(accelX))
    print("Min Acceleration Y:  ", min(accelY))
    print("Min Acceleration Z:  ", min(accelZ))

    print("Average Acceleration X:  ", sum(accelX)/len(accelX))
    print("Average Acceleration Y: ", sum(accelY)/len(accelY))
    print("Average Acceleration Z: ", sum(accelZ)/len(accelZ)) 

    print("Maximum Vertical Acceleration:  ", max(accelVertical))
    print("Minimum Vertical Acceleration:  ", max(accelVertical))
    print("Average Vertical Acceleration:  ", sum(accelVertical)/len(accelVertical))

    print("Max Velocity X: ", max(velX))
    print("Max Velocity Y: ", max(velY))
    print("Max Velocity Z: ", max(velZ))

    print("Min Velocity X:  ", min(velX))
    print("Min Velocity Y: ", min(velY))
    print("Min Velocity Z: ", min(velZ))

    print("Average Velocity X:", sum(velX)/len(velX))
    print("Average Velocity Y:", sum(velY)/len(velY))
    print("Average Velocity Z:", sum(velZ)/len(velZ))

    print("Maximum Vertical Velocity:  ", max(velVertical))
    print("Minimum Vertical Velocity:  ", min(velVertical))
    print("Average Vertical Velocity:  ", sum(velVertical)/len(velVertical))

    print("Max Pressure:  ", max(pressure))
    print("Min Pressure:  ", min(pressure))
    print("Average Pressure:  ", sum(pressure)/len(pressure))

    print("Max Temperature:  ", max(temperature))
    print("Min Temperature:  ", min(temperature))
    print("Average Temperature:  ", sum(pressure)/len(pressure))

    print("Max Altitude (Environmental):  ", max(altitudeE))
    print("Max Altitude (Accelerometer):  ", max(altitudeA))

    print("Min Altitude (Environmental):  ", min(altitudeE))
    print("Min Altitude (Accelerometer):  ", min(altitudeA))

    print("Average Altitude (Environmental):  ", sum(altitudeE)/len(altitudeE))
    print("Average Altitude (Accelerometer):  ", sum(altitudeA)/len(altitudeA))

    print("Max Force X:  ", max(forceX))
    print("Min Force X:  ", min(forceX))
    print("Average Force X:  ", sum(forceX)/len(forceX))

    print("Max Force Y:  ", max(forceY))
    print("Min Force Y:  ", min(forceY))
    print("Average Force Y: ", sum(forceY)/len(forceY))

    print("Max Force Z:  ", max(forceZ))
    print("Min Force Z:  ", min(forceZ))
    print("Average Force Z: ", sum(forceZ)/len(forceZ))

    print("Maximum Vertical Force:  ", max(forceVertical))
    print("Minimum Vertical Force:  ", min(forceVertical))
    print("Average Vertical Force:  ", sum(forceVertical)/len(forceVertical))

    print("Max Tilt X:  ", max(Xtilt))
    print("Min Tilt X:  ", min(Xtilt))
    print("Average Tilt X:  ", sum(Xtilt)/len(Xtilt))

    print("Max Tilt Z:  ", max(Ztilt))
    print("Min Tilt Z:  ", min(Ztilt))
    print("Avereage Tilt Z:  ", sum(Ztilt)/len(Ztilt))

    print("Max Absolute Tilt:  ", max(absoluteTilt))
    print("Min Absolute Tilt:  ", min(absoluteTilt))
    print("Average Absolute Tilt:  ", sum(absoluteTilt)/len(absoluteTilt))

    print("Launch Time:", launchTime)
    print("Descent Began:  ", descendingTime)
    print("Landing Time:  ", landedTime)
    print("Final Time:  ", realtime[-1])
    print("Average Transmission Time:  ", averageDifference(realtime))




def read_and_process_serial_data():
    print("reading")
    global beginCalcs, realtime, mode, accelX, acceY, accelZ, pressure, temperature, beginCalcs, refTemp, refPress, lists_to_process
    try:
        # Set up serial connection
        ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
        print(f"Connected to {serial_port} at {baud_rate} baud.")
        # Define the terminator and the size of the float set
        terminator = b'\xff\xff\xff\xff'
        num_floats = 7
        float_size = 4
        packet_size = num_floats * float_size  # Size of data in bytes (7 floats)

        buffer = bytearray()
        
        # if realtime[-1] - launchTime > 15:
        #     mode = 4

        while True:
            #print("true")
            if mode == 4:
                    if broadcastedFinalResults == False:
                        printData()

            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting)

                terminator_pos = buffer.find(terminator)
                while len(buffer>=packet_size):
                    while terminator_pos != -1:
                        # Ensure there is a complete packet before the terminator
                        if terminator_pos >= packet_size:
                            packet_start_pos = terminator_pos - packet_size
                            packet = buffer[packet_start_pos:terminator_pos]

                            if refTemp == 0: #Sets up reference values for the altitude formula
                                refTemp=temperature[-1]
                            if refPress == 0:
                                refPress=pressure[-1]

                            
                            try:
                                dataList = list(struct.unpack('<9B', packet)) #This unpacks the data, reading it as a series of unsigned integers


                                try:
                                    realtime.append((float((dataList[0] << 4) | (dataList[1] >> 4)))/1000)
                                except ValueError:
                                    if realtime:  # Check if not empty to avoid IndexError
                                        realtime.append(realtime[-1])

                                try:
                                    if ((dataList[1] >> 3) & 0x08) == 1:
                                        mode.append(mode[-1]+1)
                                    else:
                                        mode.append(mode[-1])
                                except ValueError:
                                    if mode:
                                        mode.append(mode[-1])



                                try:
                                    accelY.append(float(map_accel_y(((dataList[1] & 0x70) >> 4) << 7 | ((dataList[2] & 0xFE) >> 1))))

                                    if accelY[-1]<0:
                                        accelY[-1]+=g
                                    else:
                                        accelY[-1]-=g
                                except ValueError:
                                    if accelY:
                                        accelY.append(accelY[-1])


                                try:
                                    tempX = (((dataList[2] & 0x01) << 7) | (dataList[3] >> 1))
                                    if tempX > 128:
                                        tempX -= 256
                                    tempX += calib0X
                                    accelX.append(float(map_accel_x(tempX)))

                                except ValueError:
                                    if accelX:
                                        accelX.append(accelX[-1])

                                
                                try:
                                    tempZ = (((dataList[3] & 0x01) << 7) | (dataList[4] >> 1))
                                    if tempZ > 128:
                                        tempZ -= 256
                                    tempZ += calib0Z
                                    accelZ.append((float(map_accel_z(tempZ))))

                                except ValueError:
                                    if accelZ:
                                        accelZ.append(accelZ[-1])
                                
                                try:
                                    tempTemp = (((dataList[4] & 0x01) << 8) | dataList[5])
                                    if tempTemp > 512:
                                        tempTemp -= 1024
                                    tempTemp = tempTemp/1000 + calibTemp
                                    temperature.append(float(tempTemp))


                                except ValueError:
                                    if temperature:
                                        temperature.append(temperature[-1])


                                try:
                                    tempPress = (((dataList[5] & 0x03) << 12) | (dataList[6] << 4) | ((dataList[7] & 0xF0) >> 4))
                                    if tempPress > 8192:
                                        tempPress -= 16384
                                    tempPress = tempPress/1000 + seaLevelPressure
                                    pressure.append(tempPress)
                                except ValueError:
                                    if pressure:
                                        pressure.append(pressure[-1])

                                #Uses all the functions declared before
                                calculateTilt()
                                calculateVerticalAcceleration()
                                calculateMaxAcceleration()
                                calculateVelocity(accelX[-1], accelY[-1], accelZ[-1])
                                checkMode()
                                calculateForce(accelX[-1], accelY[-1], accelZ[-1])
                                checkEnvironmentalMaxs()
                                calculateEnvironmentalAltitude(pressure[-1], temperature[-1])
                                calculateAccelerometerAltitude()
                                checkModeName()
                                checkForModeChange
                                appendExcelFunc()
                                saveLists()
                                
                            except struct.error as e:
                                print(f"Error unpacking data: {e}")
                            
                            # Remove processed packet and its terminator from the buffer
                            buffer = buffer[terminator_pos + len(terminator):]
                        else:
                            # Not enough data before this terminator, remove up to and including this terminator
                            buffer = buffer[terminator_pos + len(terminator):]

                        # Look for the next terminator in the remaining buffer
                        terminator_pos = buffer.find(terminator)
                        
                        
                    time.sleep(0.5)  # For Debugging

    except serial.SerialException as e: #Error handling
        print(f"Error opening serial port: {e}")

    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    finally:
        if 'ser' in locals() or 'ser' in globals():
            #ser.close()
            print("Serial port error")


windowSize = 30 #How many seconds are displayed by the graphs

def update_limits(ax, time_data, timecheck, window_size=60):  # Adjust window_size as needed
    if timecheck > 0:
        right_limit = timecheck
        left_limit = max(0, right_limit - window_size)
        # Ensure there's a minimal difference between left_limit and right_limit
        if left_limit == right_limit:
            left_limit -= 1  # Decrement left_limit
            right_limit += 1  # Increment right_limit to ensure they are not identical
        ax.set_xlim(left_limit, right_limit)
        ax.figure.canvas.draw()

scroll_offset = 0 #Artefact of proposed manual scrolling mechanic

# def on_press(event):
#     global scroll_offset
#     if event.key == 'left':
#         scroll_offset += 1  # Scroll back in history
#     elif event.key == 'right':
#         scroll_offset = max(0, scroll_offset - 1)  # Scroll forward in history
#     # Note: You might need to refresh your plots here to apply the new scroll_offset

def start_graphing():
    global scroll_offset
    fig1, axs1 = plt.subplots(3, num='Acceleration Data')
    fig2, axs2 = plt.subplots(3, num='Environmental Data')
    fig3, axs3 = plt.subplots(3, num='Velocity Data')
    fig4, axs4 = plt.subplots(3, num='Force Data')
    fig5, axs5 = plt.subplots(3, num='Vertical Data')


    def animate_accel(i):
        try:
            for j, ax in enumerate(axs1):
                ax.clear()
                if j == 0:
                    equalized_accelX = equalize_before_plot(accelX, "accelX") #Ensures list lengths are equal to "realtime", which they are plotted against, to avoid Matplotlib errors
                    ax.plot(range(len(realtime)), equalized_accelX, label='Accel X') #Calls the Matplotlib plot() function
                    ax.set_title('Real-time Acceleration X')
                elif j == 1:
                    equalized_accelY = equalize_before_plot(accelY, "accelY")
                    ax.plot(range(len(realtime)), equalized_accelY, label='Accel Y')
                    ax.set_title('Real-time Acceleration Y')
                elif j == 2:
                    equalized_accelZ = equalize_before_plot(accelZ, "accelZ")
                    ax.plot(range(len(realtime)), equalized_accelZ, label='Accel Z')
                    ax.set_title('Real-time Acceleration Z')
                update_limits(ax, range(len(realtime)), realtime[-1], scroll_offset) #Updates the axes to give an animated effect
        except Exception as e:
            print(f"Error in animate_accel: {e}") #Prevents the graphing from ceasing should an error occur

    def animate_env(i):
        try:
            for j, ax in enumerate(axs2):
                ax.clear()
                if j == 0:
                    equalized_temp = equalize_before_plot(temperature, "temperature")
                    ax.plot(range(len(realtime)), equalized_temp, label='Temperature')
                    ax.set_title('Real-time Temperature')
                elif j == 1:
                    equalized_press = equalize_before_plot(pressure,'Pressure')
                    ax.plot(range(len(realtime)), equalized_press, label='Pressure')
                    ax.set_title('Real-time Pressure')
                elif j == 2:
                    equalized_alt = equalize_before_plot(altitudeE,'Altitude')
                    ax.plot(range(len(realtime)), equalized_alt, label='Altitude')
                    ax.set_title('Real-time Altitude')
                update_limits(ax, range(len(realtime)), realtime[-1], scroll_offset)
        except Exception as e:
            print(f"Error in animate_env: {e}")

    def animate_vel(i):
        try:
            for j, ax in enumerate(axs3):
                ax.clear()
                if j == 0:
                    equalized_velX = equalize_before_plot(velX,'Vel X')
                    ax.plot(range(len(realtime)), equalized_velX, label='Vel X')
                    ax.set_title('Real-time Velocity X')
                elif j == 1:
                    equalized_velY = equalize_before_plot(velY,'Vel Y')
                    ax.plot(range(len(realtime)), equalized_velY, label='Vel Y')
                    ax.set_title('Real-time Velocity Y')
                elif j == 2:
                    equalized_velZ = equalize_before_plot(velZ,'Vel Z')
                    ax.plot(range(len(realtime)), equalized_velZ, label='Vel Z')
                    ax.set_title('Real-time Velocity Z')
                update_limits(ax, range(len(realtime)), realtime[-1], scroll_offset)
        except Exception as e:
            print(f"Error in animate_vel: {e}")

    def animate_force(i):
        try:
            for j, ax in enumerate(axs4):
                ax.clear()
                if j == 0:
                    equalized_forceX = equalize_before_plot(forceX,'force X')
                    ax.plot(range(len(realtime)), equalized_forceX, label='force X')
                    ax.set_title('Real-time Force X')
                elif j == 1:
                    equalized_forceY = equalize_before_plot(forceY,'force Y')
                    ax.plot(range(len(realtime)), equalized_forceY, label='force Y')
                    ax.set_title('Real-time Force Y')
                elif j == 2:
                    equalized_forceZ = equalize_before_plot(forceZ,'force Z')
                    ax.plot(range(len(realtime)), equalized_forceZ, label='force Z')
                    ax.set_title('Real-time Force Z')
                update_limits(ax, range(len(realtime)), realtime[-1], scroll_offset)
        except Exception as e:
            print(f"Error in animate_force: {e}")

    def animate_vert(i):
        try:
            for j, ax in enumerate(axs5):
                ax.clear()
                if j == 0:
                    equalized_vel = equalize_before_plot(velVertical,'Vel Vertical')
                    ax.plot(range(len(realtime)), equalized_vel, label='Vel Vertical')
                    ax.set_title('Real-time Vertical Velocity')
                elif j == 1:
                    equalized_accel = equalize_before_plot(accelVertical,'Accel Vertical')
                    ax.plot(range(len(realtime)), equalized_accel, label='Accel Vertical')
                    ax.set_title('Real-time Vertical Acceleration')
                elif j == 2:
                    equalized_force = equalize_before_plot(forceVertical,'Force Vertical')
                    ax.plot(range(len(realtime)), equalized_force, label='Force Vertical')
                    ax.set_title('Real-time Vertical Force')
                update_limits(ax, range(len(realtime)), realtime[-1], scroll_offset)
        except Exception as e:
            print(f"Error in animate_vert: {e}")

    # Setup animations
    ani1 = FuncAnimation(fig1, animate_accel, interval=50, cache_frame_data=False)
    ani2 = FuncAnimation(fig2, animate_env, interval=50, cache_frame_data=False)
    ani3 = FuncAnimation(fig3, animate_vel, interval=50, cache_frame_data=False)
    ani4 = FuncAnimation(fig4, animate_force, interval=50, cache_frame_data=False)
    ani5 = FuncAnimation(fig5, animate_vert, interval=50, cache_frame_data=False)

    plt.show()



def setupFunc():
    seri = serial.Serial(serial_port, baud_rate, timeout=timeout)
    print(f"Connected to {serial_port} at {baud_rate} baud.")
    # message = "BEGIN"

    def waitForMessage(message, expected_message, firsttimeout = 1, timeout_duration=5, reset_message="R", reset_ack="R"):
        start_time = time.time()
        secondtime = time.time()
        while True:
            if seri.in_waiting:
                line = seri.readline().decode('utf-8', errors='ignore').rstrip()
                if expected_message in line:
                    return "SUCCESS"
                elif reset_ack in line:  # Handle receiving reset acknowledgment
                    print("Reset acknowledged. Starting calibration over.")
                    return "RESET"
            else:
                current_time = time.time()
                
                if (current_time - start_time) > timeout_duration:
                    seri.write(reset_message.encode())  # Send RESET command
                    print("No response, sending RESET.")
                    start_time = current_time  # Reset the start time for the next interval
                    timeout_duration = 2  # Set the new timeout duration for RESET handling
                elif ((current_time-secondtime) > firsttimeout) and ((current_time-start_time) < timeout_duration):
                    print("No response, retrying")
                    seri.write(message.encode())
                    secondtime = current_time

    def waitForReset(timeoutDuration = 2,reset_message="R", reset_ack="R"):
        start_time = time.time()
        while True:
            if seri.in_waiting:
                line = seri.readline().decode('utf-8', errors='ignore').rstrip()
                if reset_ack in line:  # Handle receiving reset acknowledgment
                    print("Reset acknowledged. Starting calibration over.")
                    return "RESET"
            else:
                current_time = time.time()
                
                if (current_time - start_time) > timeoutDuration:
                    seri.write(reset_message.encode())  # Send RESET command
                    print("Sending RESET.")
                    start_time = current_time  # Reset the start time for the next interval

    def findCalibValues():
        global identifier, calibPosX, calib0X, calibNegX, calibPosY, calib0Y, calibNegY, calibPosZ, calib0Z, calibNegZ, calibTemp
        line = seri.readline().decode('utf-8').strip()

        # Assuming the line format is as you described, with commas separating values
        # Check if the line is not empty
        if line:
            # Split the line into values
            values = line.split(",")

            # Ensure that there are enough values in 'values' to unpack successfully

            if len(values) == 10:
                identifier, calibPosX, calib0X, calibNegX, calibPosY, calib0Y, calibNegY, calibPosZ, calib0Z, calibNegZ, calibTemp = values
            
            else:
                waitForReset()

        else:
                waitForReset()

    def calibrationStep(instruction, message, expectedMessage, success_message):
        while True:
            numberIn = int(input(instruction))
            if numberIn == 1:
                if message == "C7":
                    findCalibValues
                seri.write(message.encode())
                response = waitForMessage(message, expectedMessage)
                if response == "SUCCESS":
                    print(success_message)
                    return True
                elif response == "RESET":
                    return False
            elif numberIn == 404:
                response = waitForReset()
                if response == "RESET":
                    return False
            else:
                print("Input not recognised. Try again.")

    while True:

        ChoiceNum = input("Use old calibration values (1), recalibrate (2), or read pre-initialised data-stream (3):   ")

        if ChoiceNum == "1":
            if calibrationStep("Enter 1 to confirm choice:  ", "OLD", "X",
                                   "Calibration Complete, Data Transmission Has Begun"):
                break
            else:
                continue

        elif ChoiceNum == "2":

            if not calibrationStep("Please orient Arduino on the Z-axis. Enter '1' to confirm Z-axis calibration (Note: at any point you can input '404' to restart calibration):   ", "C1","C",
                                "Z Calibration 1 Complete. Please Invert Arduino."):
                continue  # Restart calibration if RESET is acknowledged
            if not calibrationStep("Enter '1' to confirm inverted Z-axis calibration:   ","C2","C",
                                "Z Calibration 2 Complete. Please orient Arduino on X-axis."):
                continue
            if not calibrationStep("Enter '1' to confirm X-axis calibration:   ","C3","C",
                                "X Calibration 1 Complete. Please Invert Arduino."):
                continue
            if not calibrationStep("Enter '1' to confirm inverted X-axis calibration:   ","C4","C",
                                "X Calibration 2 Complete. Please orient Arduino on Y-axis."):
                continue
            if not calibrationStep("Enter '1' to begin Y-axis (vertical) calibration:   ","C5","C",
                                "Y Calibration 1 Complete. Please Invert Arduino."):
                continue
            if not calibrationStep("Enter '1' to begin inverted Y-axis calibration:   ","C6","C",
                                "Y Calibration 2 Complete. Good job!"):
                continue
            if calibrationStep("Enter '1' to begin data transmission:   ","C7","C",
                            "Data Transmission Has Begun"): #This transmission is important, as it triggers the listening for calibration values
                break #exit the loop if all previous steps have been executed sequentially without a "false" being returned
            else:
                continue

        elif ChoiceNum == "3":
            confirmNum = input("Enter 1 to confirm choice:  ")
            if confirmNum == "1":
                break
            else:
                continue





def command_endGraph():
    print("End Graph command called")
    plt.close('all')  # Close all Matplotlib plots
    # Additional commands to end graphing processes if necessary

def command_endTransmission():
    # Signal the serial plotting thread to stop
    stop_event.set()
    print("Serial transmission ending requested.")

def command_printData():
    printData()

def handle_user_input():
    while True:
        user_input = input()
        if user_input == "endGraph":
            command_endGraph()
        elif user_input == "print":
            print("Printing")
            command_printData()
        elif user_input == "endTransmission":
            command_endTransmission()
            break  # Exit the loop to end the program, or remove break if you want to keep the program running
        elif user_input == "END ALL":
            command_endGraph()
            command_endTransmission()
            break  # Exit the loop to end the program
    print("User input handler stopped.")


if __name__ == "__main__":
    setupFunc()
    #Start the serial data handling in a separate thread
    serial_thread = threading.Thread(target=read_and_process_serial_data, daemon=True)
    serial_thread.start()

    # Start the graphing on the main thread
    start_graphing()
    handle_user_input()


    time.sleep(5)
    graphAfter()
