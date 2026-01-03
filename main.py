import board
import numpy as np
import pwmio
import pulseio
from digitalio import DigitalInOut
from digitalio import Direction
import adafruit_bmp3xx
import adafruit_pca9685
import measure_servofreq_adafruit_pca9685
import time
import busio
import adafruit_bno055
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
bmp.sea_level_pressure = 1032
#print("help")
#Servo Freq Function
def measure_servofreq(servo_breakout_object,servo_channel_index,measurement_pin):
    """Measure servo pulse repetition frequency. Assumes servo output 
    corresponding to channel_index is attached through a ~2.2k resistor 
    to the specified measurement_pin. Returns the typical (median) 
    pulse frequency. The servo breakout should already be configured 
    to generate pulses. 

    Note that this creates a temporary pulsein object that monitors the
    specified pin"""
    
    n_periods_to_measure = 51 # Must be odd

    periods = [] # measured in microseconds
    
    pulse_measure = pulseio.PulseIn(measurement_pin,maxlen=6,idle_state=False)
    for period_cnt in range(n_periods_to_measure):
        while len(pulse_measure) < 1:
            pass
        
        positive_width = pulse_measure.popleft()
        
        while len(pulse_measure) < 1:
            pass

        negative_width = pulse_measure.popleft()

        periods.append(positive_width + negative_width)

        pass

    pulse_measure.deinit() # Don't need this anymore

    # evaluate median
    # sort periods
    periods.sort()

    assert(len(periods)==n_periods_to_measure)
    assert(n_periods_to_measure % 2 == 1) # odd

    median_period = periods[(n_periods_to_measure-1)//2]  # in microseconds
    frequency = 1.0e6/median_period

    return frequency
#End of servofreq definition
#start of get_pulse_commands def

def get_pulse_commands(pulsein_objs):
    num_inputs = len(pulsein_objs)
    latests=[ None ]*num_inputs

    all_complete = False
    while not all_complete:

        all_complete=True
        for i in range(num_inputs):
            if len(pulsein_objs[i]) > 0:
                new_us = pulsein_objs[i].popleft()
                #print("got pulse[%d] new_us=%f" % (i,new_us))
                if new_us > 900 and new_us < 2100:
                    latests[i] = new_us
                    pass
                pass
            if latests[i] is None:
                all_complete = False
                pass
            
            pass
        
        pass
    return latests


#end of get pulse commands function    
board.lvez1_uart="pyserial"

import serial
import threading
import code
def run_repl():
 code.interact(local=globals())
 pass
threading.Thread(target=run_repl).start()

commandtime = time.monotonic()

i2c = busio.I2C(board.SCL, board.SDA) #Lab 4.2
sensor = adafruit_bno055.BNO055_I2C(i2c)

led=DigitalInOut(board.D13) #Lab 3.1
led.direction = Direction.OUTPUT
aileron = pulseio.PulseIn(board.D5)  #Lab 3.3
maxlen = 1
altitude_inputPulse = pulseio.PulseIn(board.D6)
Proportionalcoefficient = 0.01
Integralterm = .1
DerivativeTerm = 0.1
IntegratedError=0
#print(altitude_inputPulse)
#Altitude Hold Declarations
# PID coefficients for altitude 
kp = .4   #actually .4
kd = .3 

integral = 0
dt_goal = 0.1
tmax = 3
t = np.arange(tmax/dt_goal) * dt_goal
b = np.cos(t * np.pi/(2*tmax))
a = np.zeros(t.shape[0], dtype = 'd') #closest is most recent


ledpwm = pwmio.PWMOut(board.D13)  #Lab 3.1 *duty cycle 1500 microseconds

servo_breakout = adafruit_pca9685.PCA9685(i2c)
servo_breakout.frequency = int(40)
servo_breakout.channels[15].duty_cycle = int(1e-3*servo_breakout.frequency*65536.0)
LIFT_PWM = servo_breakout.channels[11]
LIFT_PWM2 = servo_breakout.channels[12]
actual_frequency = measure_servofreq(servo_breakout, 15, board.D26) #ask about what to do with this actual pulse?

LIFT_PWM.duty_cycle = int(2621) 
LIFT_PWM2.duty_cycle = int(2621) 
    
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.calibration_status
#print(actual_frequency)
LIFT_PWM.duty_cycle = int(actual_frequency * 1e-3 * 65535)
sensor.mode = adafruit_bno055.ACCONLY_MODE
sensor.accel_bandwidth = int(7.81)
avg_altitude = 0
avg_gravity = 0
number_of_averages=100
for num in range(number_of_averages):
    avg_altitude += bmp.altitude
    new_acc=sensor.acceleration[2]
    while(new_acc is None)or(new_acc >= 12)or(new_acc<=8):
        new_acc=sensor.acceleration[2]
    avg_gravity += new_acc
    time.sleep(.03)
avg_altitude = avg_altitude / number_of_averages
avg_gravity = avg_gravity / number_of_averages
#This code is for getting Pi to control craft
while True:
    (commanded_altitude,) = get_pulse_commands([altitude_inputPulse]) #No idea on this 
    #print("Atitude Pulse Input =", commanded_altitude)
    commanded_altitude =  (commanded_altitude * .01) - 10
    current_altitude = bmp.altitude - avg_altitude + 1 #change as needed
    #print("Acceleration:",format(sensor.acceleration))
    new_acc=sensor.acceleration[2]
    while(new_acc is None)or(new_acc >= 12)or(new_acc<=8):
        new_acc=sensor.acceleration[2]
    acceleration = new_acc
    # PID calculations
    error = commanded_altitude - current_altitude
    #print("Commanded:", commanded_altitude)
    #print("Current:", current_altitude)
    a = np.roll(a,1)
    a[0] = acceleration - avg_gravity
    #print('a =',a)
    #print('a.shape =', a.shape)
    #print('b.shape =', b.shape)
    velocity = np.inner(a,b) * dt_goal 
    #print('velocity.shape =', velocity.shape)
    #print('velocity =', velocity)
    lift_adjustment = kp * error - kd * velocity
    lift_adjustment = max(min(lift_adjustment, 1), 0)
    
    lifthold = (1 + lift_adjustment) #Changed LIFTCMD to 1.5
    #print(f"Mid Lifthold, LIFTCMD: {lifthold}")
    #print(f"Error: {error}")
    #print("Lifth adjustment", lift_adjustment) 
    LIFT_PWM.duty_cycle = int((lifthold) * actual_frequency*65536/1000) 
    LIFT_PWM2.duty_cycle = int((lifthold) * actual_frequency*65536/1000) 
    
    #print("Duty Cycle:", LIFT_PWM.duty_cycle)
    #print('lifthold = ',lifthold)
    #print(lift_adjustment)
    time.sleep(0.1)
