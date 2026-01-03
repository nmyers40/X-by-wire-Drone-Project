import pulseio

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

    
