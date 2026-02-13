#
# File: monitoring_system.py
# Revision: v1.0
# Author: Lucas Kindermann
# Date: November 13th 2025
#
# Application: Small Monitoring System
# Hardware: Raspberry Pi Pico RP2040
# IDE: Thonny 4.1.7
# Interpreter: MicroPython (Raspberry Pi Pico)
# Additional Package: micropython-bme280 v2.1.3
#
# Behavior:
# * Collect Temperature, Pressure and Distance values from sensors (Sense).
# * Send the data to a smartphone through Bluetooth protocol and print in IDE terminal.
# * Uses Buzzer and LED as indicators when collecting & sending data.
# * Run Sense periodically, when a button is pressed or when a object is close.
#
# Notes:
# * Some adjustments can be made changing the values in the Parameterized Constants section
# * Check application documents for more info such as: diagrams, setup assemble, parts used and more...
#
##########################
## Libraries & Packages ##
##########################
from machine import Pin, I2C, UART, PWM, Timer
from utime import sleep, sleep_us, ticks_us
import bme280    

#############################
## Parameterized Constants ##
#############################
sense_time_s = 6
led_time_s = 1
buzz_time_s = 0.2
dist_check_time_s = 0.2
dist_enable_time_s = 2
dist_limit_cm = 15
buzz_freq_hz = 440
buzz_duty_pc = 10

##########################
## Calculated Constants ##
##########################
buzz_dc_u16 = round(65535 * (buzz_duty_pc/100))
dist_factor = 17165 / 1000000

#########################
## Board Configuration ##
#########################
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
bme = bme280.BME280(i2c=i2c)
buzz = PWM(Pin(13))
btn = Pin(14, Pin.IN, Pin.PULL_DOWN)
led = Pin(15, Pin.OUT)
echo = Pin(16, Pin.IN, Pin.PULL_DOWN)   
trig = Pin(17, Pin.OUT)

###########################
## Functions & Callbacks ##
###########################
def dist_check():
  ## Check distance ##
  global dist_cm
  trig.value(1)
  sleep_us(2)
  trig.value(0)
  while echo.value() == 0:
    pulse_start = ticks_us()    
  while echo.value() == 1:
    pulse_end = ticks_us()
  pulse_length = pulse_end - pulse_start
  dist_cm = round(pulse_length * dist_factor,1)

def led_off(tim_led):
  ## Turn OFF LED ##
  led.value(0)

def buzz_off(tim_buzz):
  ## Turn OFF Buzzer ##
  buzz.duty_u16(0)

def dist_on(tim_dist):
  ## Turn ON Distance under limit trigger ##
  global dist_irq_en
  dist_irq_en = 1

def sense():
  ## Get data from sensors ##
  temp_C = bme.values[0]
  pressure_hPa = bme.values[1]
  dist_check()
  
  ## Print messages to terminal and send through UART ##
  temp_msg = "\n\nTemperature: " + temp_C
  pres_msg = "\nPressure: " + pressure_hPa
  dist_msg = "\nDistance: " + str(dist_cm) + " cm"
  sense_msg = temp_msg + pres_msg + dist_msg
  uart.write(sense_msg)
  print(sense_msg)
  
  ## LED & Buzzer notification ##
  led.value(1)
  buzz.duty_u16(buzz_dc_u16)
  tim_led = Timer(mode=Timer.ONE_SHOT, period=int(led_time_s*1000), callback=led_off)
  tim_buzz = Timer(mode=Timer.ONE_SHOT, period=int(buzz_time_s*1000), callback=buzz_off)

def run_sense(tim_sense):
  ## Collect & send data when sense interval is reached ##
  sense()

def btn_handler(Pin):
  ## Collect & send data when button is pressed ##
  global tim_sense
  tim_sense.deinit()
  sense()
  tim_sense = Timer(mode=Timer.PERIODIC, period=int(sense_time_s*1000), callback=run_sense)

#############################################
## IRQ Settings & Variables Initialization ##
#############################################
btn.irq(trigger=Pin.IRQ_FALLING, handler=btn_handler)
buzz.freq(buzz_freq_hz)
tim_sense = Timer(mode=Timer.PERIODIC, period=int(sense_time_s*1000), callback=run_sense)
dist_irq_en = 1
dist_check()

##################
## Main routine ##
##################
while True:
  if dist_irq_en == 1 and dist_cm <= dist_limit_cm:
    ## Collect & send data when below the distance limit ##
    tim_sense.deinit()
    sense()
    dist_irq_en = 0
    tim_dist = Timer(mode=Timer.ONE_SHOT, period=int(dist_enable_time_s*1000), callback=dist_on)
    tim_sense = Timer(mode=Timer.PERIODIC, period=int(sense_time_s*1000), callback=run_sense)
  else:
    dist_check()
    sleep(dist_check_time_s)

