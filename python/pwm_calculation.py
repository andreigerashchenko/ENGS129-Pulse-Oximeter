def pwm_pulseox(frequency, red, black):
    period = 1000000 / frequency
    state1_time = period * red / 100
    state3_time = period * black / 100
    state2_time = period * 0.5 - state1_time
    state4_time = period * 0.5 - state3_time

    print(f"Frequency: {frequency} Hz, Period: {period} us, Red: {red}%, Black: {black}%, State 1: {state1_time} us, State 2: {state2_time} us, State 3: {state3_time} us, State 4: {state4_time} us, Total: {state1_time + state2_time + state3_time + state4_time} us")

if __name__ == "__main__":
    pwm_pulseox(1000, 10, 35)