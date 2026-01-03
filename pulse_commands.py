
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


if __name__=="__main__":
    import sys
    import time
    import board
    import pulseio
    import adafruit_bno055
    import busio
    import math
    import digitalio

    
    
    ROTCMD = pulseio.PulseIn(board.D5,maxlen=8,idle_state=False)
    
    pulses = []
    max_pulses=1000
    
    while len(pulses) < max_pulses:
        pulses.append(get_pulse_commands([ROTCMD])[0])
        
        if len(pulses) % 100 == 0:
            print("%d/%d" % (len(pulses),max_pulses))
            pass
        pass
    
    from matplotlib import pyplot as plt
    plt.hist(pulses,15)
    plt.show()

    pass
