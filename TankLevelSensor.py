#!/usr/bin/env python3
# -*- coding: utf-8 -*-

##################################################################################
# Tank Level sensor with Waterproof Ultrasonic Module JSN-SR04T                  #
# Keep attention: The Tank must be as cubic as possible, otherwise there is      #
# some additional programming needed to avoid measorement faults.                #
# Functionality:
# On triggering the Script the Sensor gets triggered and makes 20 Measurements,  #
# calculates the average and prints the result.                                  #
# The Script is designed for Python 3 on RaspberryPi                             #
##################################################################################
# Adapted by Sven Gaechter, info@allround-service.biz, published under https://github.com/sgaechter/TankLevelSensor.git
# Credits: originally coded by Bracew, published on https://forum-raspberrypi.de/forum/thread/7428-heizoel-tankstand-oder-verbrauchs-fernablesung-mit-raspi-geloest-beitrag-21/?pageNo=2

# import required modules
import time
import datetime
import RPi.GPIO as GPIO


# define GPIO pins
GPIOTrigger = 18
GPIOEcho    = 17


# function to measure the distance
def MeasureDistance():
  time.sleep(0.2)
  GPIO.output(GPIOTrigger, True)


  # set trigger after 10µs to low
  time.sleep(0.0001)
  GPIO.output(GPIOTrigger, False)


  # store initial start time
  StartTime = time.time()


  # store start time
  while GPIO.input(GPIOEcho) == 0:
    StartTime = time.time()


  # store stop time
  while GPIO.input(GPIOEcho) == 1:
    StopTime = time.time()

# To examine the max Level check the Level of the Liquid when the tank ist fully filled (ex. 4500liters).
maxlevel = 150

# max filling quantity of the tank in Liter
maxFillment = 4500


  # Calculate distance by regarding the average waves locomotion in the air at 20 degrees
  TimeElapsed = StopTime - StartTime
  Distance = (TimeElapsed * 34400) / 2

  return Distance


# main function
def main():
  try:
#    while True:
      Distance0 = MeasureDistance()
      Distance01 = MeasureDistance()
      Distance02 = MeasureDistance()
      Distance03 = MeasureDistance()
      Distance04 = MeasureDistance()
      Distance05 = MeasureDistance()
      Distance06 = MeasureDistance()
      Distance07 = MeasureDistance()
      Distance08 = MeasureDistance()
      Distance09 = MeasureDistance()
      Distance10 = MeasureDistance()
      Distance11 = MeasureDistance()
      Distance12 = MeasureDistance()
      Distance13 = MeasureDistance()
      Distance14 = MeasureDistance()
      Distance15 = MeasureDistance()
      Distance16 = MeasureDistance()
      Distance17 = MeasureDistance()
      Distance18 = MeasureDistance()
      Distance19 = MeasureDistance()
      Distance20 = MeasureDistance()
      Distance_sum = Distance01 + Distance02 + Distance03 + Distance04 + Distance05 + Distance06 + Distance07 + Distance08 + Distance09 + Distance10 + Distance11 + Distance12 + Distance13 + Distance14 + Distance15 + Distance16 + Distance17 + Distance18 + Distance19 + Distance20
      Distance = round(Distance_sum / 20,1)

      Level = maxlevel - Distance
      liter = maxFillment / maxlevel * Level
      Time = time.time()
      Timestamp = datetime.datetime.fromtimestamp(Time).strftime('%Y-%m-%d_%H:%M:%S')
      print((Timestamp),("Distance: %.1f cm" % Distance),(" Level: %.1f cm" % Level),(" liter: %.0f l" % liter))
      time.sleep(2)


  # reset GPIO settings if user pressed Ctrl+C
  except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()


if __name__ == '__main__':
  # use GPIO pin numbering convention
  GPIO.setmode(GPIO.BCM)


  # set up GPIO pins
  GPIO.setup(GPIOTrigger, GPIO.OUT)
  GPIO.setup(GPIOEcho, GPIO.IN)


  # set trigger to false
  GPIO.output(GPIOTrigger, False)


  # call main function
  main()