#!/usr/bin/pyth
#Importing functions others wrote in order to acess those programs
import sqlite3
import time
import datetime
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import glob
import os
import adafruit_gps
import serial

def main():
 debugflag= False

 os.system('modprobe w1-gpio')
 os.system('modprobe w1-therm')

 base_dir = '/sys/bus/w1/devices/'
 device_folder = glob.glob(base_dir + '28*')[0]
 device_file = device_folder + '/w1_slave'

 #Tempurature sensor read function
 def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines
 #Opening and reading file connected to device
 def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        #temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c
 #This grabs the information out of the device and converts it into numbers we understand

 #Tubidity conversion
 def read_turb(temp2):
    rawturb = chan2.voltage
    if (debugflag):
        print (rawturb)
    if(rawturb >= 2.5):
        rawturb2 = rawturb + (temp2 - 30)*.01275
        turbidity = -1120.4*pow(rawturb2,2)+5742.3*rawturb2-4352.9-750
    else:
        turbidity = 3100.0
    return turbidity, rawturb

 #PH sensor conversion
 def read_PH():
    rawPH = chan0.voltage
    if (debugflag):
        print (rawPH)
    PH = 0.7956*pow(rawPH,2)-10.06*rawPH+27.38
    return PH, rawPH

 def read_GPSfix():
     gps.update()
     if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print('Waiting for fix...')
            return 0
     else:
         return 1

 def read_GPS():
     gps.update()
     GPSfix = gps.has_fix
     if not gps.has_fix:
         return datetime.datetime.now(), "44.37705, -73.09002", GPSfix
     else:
         GPSdatime =('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
            gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
            gps.timestamp_utc.tm_mday,  # struct_time object that holds
            gps.timestamp_utc.tm_year,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,   # month!
            gps.timestamp_utc.tm_sec)) 
         GPSPosition = ('Latitude: {0:.6f} '.format(gps.latitude)) + ('Longitude: {0:.6f} '.format(gps.longitude))
         return GPSdatime, GPSPosition, GPSfix
      
 
 #RX = board.RX
 #TX = board.TX
 
 uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=3000)
 gps = adafruit_gps.GPS(uart, debug=False)
 gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
 gps.send_command(b'PMTK220,1000')
 ## setup input pins
 GPIO.setmode(GPIO.BCM)
 GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
 GPIO.setup(24, GPIO.OUT)
 # Create the I2C bus
 i2c = busio.I2C(board.SCL, board.SDA)

 # Create an ADS1115 ADC (16-bit) instance.
 ads = ADS.ADS1015(i2c)

 if (debugflag):
        print('Reading ADS1x15 values, press Ctrl-C to quit...')
        # Print nice channel column headers.
        print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
        print('-' * 37)

 chan0 = AnalogIn(ads, ADS.P0)
 chan1 = AnalogIn(ads, ADS.P1)
 chan2 = AnalogIn(ads, ADS.P2)
 #chan3 = AnalogIn(ads, ADS.P3)

 oktoread = False
 okforpin = True
 testGPS = read_GPSfix()
 for i in range(6):   # try upto 6 times to get gps setup
     testGPS = read_GPSfix()
     if (testGPS == 0):
         print("No GPS Fix")
     else:
         print("GPS on")
         oktomeas = True
         break
         
 oktomeas = True
 #adcchannels = [chan0, chan1, chan2, chan3]
 # Main loop.

 while oktomeas:
  while (okforpin):
    GPIO.output(24, 1)
    GPIO.wait_for_edge(23, GPIO.RISING)
    oktoread = True
    okforpin = False
    if (debugflag):
      testGPS = read_GPSfix()
      if (testGPS == 0):
          print("No GPS Fix")
    time.sleep(0.2)
    #datime = datetime.datetime.now()
    #if (debugflag):
    #    print()

  while (oktoread):
      gps.update()
      try:
          db = sqlite3.connect('/home/pi/projects/test2.db')
          cursor = db.cursor()
          cursor.execute('''CREATE TABLE IF NOT EXISTS LAKEDATA
             (
             DATETIME           TEXT    NOT NULL,
             PHSENSOR            REAL,
             PHSENSOR_RAW        REAL,
             TEMPERATURE        REAL,
             TURBIDITY         REAL,
             TURBIDITY_RAW         REAL,             
             GPS               TEXT    NOT NULL);''')
          db.commit()
      except:
          if (debugflag):
              print('database not found')
          exit()
      rawvalues = [0,0,0]
      #values[0], rawvalues[0] = read_PH()
      #values[1] = read_temp()
      #values[2], rawvalues[2]  = read_turb()
      values = [0,0,0]
      phave=[]
      tempave=[]
      turbave=[]
      testGPS =0
      trygpscnt=0
      while(testGPS ==0):
          testGPS = read_GPSfix()
          trygpscnt =+1
          if (testGPS == 0):
              print("No GPS Fix")
          time.sleep(0.2)
          if (trygpscnt >= 6):
              break
      for i in range(6):
          time.sleep(0.3)
          datime, gpslocal, gpsfixed = read_GPS()
          ph,rph = read_PH()
          temp = read_temp()
          tb, rtb = read_turb(temp)
          phave.append(ph)
          tempave.append(temp)
          turbave.append(tb)
          rawvalues[0] = rph
          rawvalues[2] = rtb
      values[0] = (phave[0] + phave[1] + phave[2] + phave[3] + phave[4] + phave[5])/6
      values[1] = (tempave[0] + tempave[1] + tempave[2] + tempave[3] + tempave[4] + tempave[5])/6
      values[2] = (turbave[0] + turbave[1] + turbave[2] + turbave[3] + turbave[4] + turbave[5])/6
      #values[3] = chan3.voltage
      #gpslocal = "44.37705, -73.09002"
      oktoread = False
      okforpin = True
      if (debugflag):
           print('| {0:>6} | {1:>6} | {2:>6} |'.format(*values))
           print('| {0:>6} | {1:>6} | {2:>6} |'.format(*rawvalues))
           print('GPS date {}' .format(datime))
           print('GPS loca {}' .format(gpslocal))
      time.sleep(0.5)
      print("Measure Done")
      try:
            cursor.execute("insert into LAKEDATA values (?, ?, ?, ?, ?, ?, ?)", (datime, values[0], rawvalues[0], values[1], values[2], rawvalues[2], gpslocal))
            db.commit()
            #db.close()
      except:
            if (debugflag):
                print('db write error??')
      #oktomeas = False
      
if __name__ == "__main__":
    main()
    