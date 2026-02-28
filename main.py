import machine
import time
import math
import struct
import rp2
from machine import Pin
import bme280  # Gebruik de externe module op je RP2350

# --- 1. CONFIGURATIE ---
LAUNCH_G_THRESHOLD = 1.3
LOG_DURATION_MAX = 60     
SAMPLE_RATE_MS = 50       
LANDING_IDLE_TIME = 2500  

# --- 2. WS2812 PIO LED (Pin 16) ---
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1, T2, T3 = 2, 5, 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(16))
sm.active(1)

def set_led(r, g, b):
    sm.put((int(r) << 24) | (int(g) << 16) | (int(b) << 8))

def pulse_led(r_max, g_max, b_max):
    brightness = (math.sin(time.ticks_ms() * 0.005) + 1) / 2
    set_led(r_max * brightness, g_max * brightness, b_max * brightness)

# --- 3. BMI160 DRIVER ---
class BMI160_Logic:
    def __init__(self, i2c, addr=0x69):
        self.i2c, self.addr = i2c, addr
        self.i2c.writeto_mem(addr, 0x7E, b'\xb6') 
        time.sleep(0.1)
        self.i2c.writeto_mem(addr, 0x7E, b'\x11') 
        time.sleep(0.1)
        self.i2c.writeto_mem(addr, 0x41, b'\x0c') # 16G
    def get_accel_g(self):
        d = self.i2c.readfrom_mem(self.addr, 0x12, 6)
        ax, ay, az = struct.unpack('<hhh', d)
        return math.sqrt((ax/2048)**2 + (ay/2048)**2 + (az/2048)**2)

# --- 4. MISSION CONTROL ---
i2c = machine.I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)

def main():
    try:
        print("\n--- INITIALISEREN SENSOREN ---")
        bmi = BMI160_Logic(i2c)
        # Initialiseer de bme280 module
        bme_sensor = bme280.BME280(i2c=i2c, address=0x76)
        
        # FASE 1: KALIBRATIE
        print("FASE 1: Nauwkeurige kalibratie (10 sec)...")
        acc_s, press_s = [], []
        start_cal = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_cal) < 10000:
            pulse_led(100, 0, 0)
            # Haal de druk op via de bme280 module (meestal hPa)
            p_str = bme_sensor.values[1].replace('hPa', '')
            press_s.append(float(p_str))
            acc_s.append(bmi.get_accel_g())
            time.sleep(0.1)
        
        acc_s.sort(); press_s.sort()
        acc_s.pop(0); acc_s.pop(-1); press_s.pop(0); press_s.pop(-1)
        bias_g = sum(acc_s) / len(acc_s)
        ground_p = sum(press_s) / len(press_s)
        
        # Bereken grondhoogte referentie
        ground_alt = 44330 * (1 - (ground_p / 1013.25)**(1/5.255))
        print(f"Kalibratie OK. Gronddruk: {ground_p:.2f} hPa")

        # FASE 2: STANDBY
        print("FASE 2: Standby. Wachten op lancering...")
        while True:
            pulse_led(0, 100, 0)
            if (bmi.get_accel_g() - bias_g) > LAUNCH_G_THRESHOLD: break
            time.sleep(0.01)

        # FASE 3: VLUCHT
        print("FASE 3: LANCERING!")
        set_led(0, 0, 200)
        start_ms = time.ticks_ms()
        data, max_alt = [], 0
        last_move_ms = time.ticks_ms()
        
        while time.ticks_diff(time.ticks_ms(), start_ms) < LOG_DURATION_MAX * 1000:
            ms = time.ticks_diff(time.ticks_ms(), start_ms)
            
            # Gebruik de bme280 module voor de druk
            p_now = float(bme_sensor.values[1].replace('hPa', ''))
            curr_abs = 44330 * (1 - (p_now / 1013.25)**(1/5.255))
            alt_rel = curr_abs - ground_alt
            
            if alt_rel > max_alt: max_alt = alt_rel
            
            g_net = bmi.get_accel_g() - bias_g
            data.append((ms, alt_rel, g_net))
            
            if len(data) % 20 == 0:
                print(f"T+ {ms/1000:.1f}s | {alt_rel:.2f}m | {g_net:.2f}G")

            if abs(g_net) > 0.15: 
                last_move_ms = time.ticks_ms()
            
            if ms > 3000 and time.ticks_diff(time.ticks_ms(), last_move_ms) > LANDING_IDLE_TIME:
                print("Landing gedetecteerd.")
                break
                
            time.sleep_ms(SAMPLE_RATE_MS)

        # FASE 4: OPSLAAN
        print("FASE 4: Opslaan...")
        set_led(0, 50, 0)
        with open("flight_log.csv", "w") as f:
            f.write("tijd_ms,hoogte_m,versnelling_g\n")
            for t, a, g in data: f.write(f"{t},{a:.2f},{g:.2f}\n")
            f.write(f"\nSAMENVATTING\nApogeum_m,{max_alt:.2f}\n")
        
        set_led(0, 255, 0)
        print(f"Missie voltooid! Apogeum: {max_alt:.2f} m")
        while True: time.sleep(1)

    except Exception as e:
        print(f"Fout: {e}")
        while True: set_led(255, 0, 0); time.sleep(0.1); set_led(0, 0, 0); time.sleep(0.1)

if __name__ == "__main__":
    main()

