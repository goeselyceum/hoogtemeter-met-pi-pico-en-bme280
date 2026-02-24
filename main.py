"""
Waterraket Hoogtemeter voor Raspberry Pi Pico
Hardware:
  - BME280 op I2C: SDA=pin0, SCL=pin1, VCC=pin2 (power pin)
  - NeoPixel op pin23

Status LED kleuren:
  Blauw pulserend  = kalibreren (baseline meting)
  Geel knipperend  = klaar voor lancering (wacht op lancering)
  Groen            = meten (vlucht actief)
  Paars knipperend = CSV opslaan
  Wit              = klaar / opgeslagen

Automatische lanceerdetectie:
  Als de hoogte > LAUNCH_THRESHOLD meter stijgt tov baseline -> start logging
  Als hoogte daalt tot <= LAND_THRESHOLD en was al gestegen -> vlucht voorbij
"""

import machine
import time
import math
import struct
import neopixel

# ── Pinnen ──────────────────────────────────────────────────────────────────
BME_SDA_PIN  = 0
BME_SCL_PIN  = 1
BME_VCC_PIN  = 2
NEO_PIN      = 23

# ── Instellingen ─────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ      = 20          # 20 metingen per seconde
SAMPLE_INTERVAL_MS  = 1000 // SAMPLE_RATE_HZ   # 50 ms

ALT_SMOOTH_N = 5
alt_history = []

CALIB_WARMUP 		= 10  		  # eerste 10 samples negeren
CALIB_SAMPLES       = 50          # aantal samples voor baseline kalibratie
LAUNCH_THRESHOLD    = 1.8         # meter boven baseline om lancering te detecteren
LAND_THRESHOLD      = 2.5         # meter boven baseline; als we hieronder zakken = geland
MIN_FLIGHT_SAMPLES  = 20          # minimale samples voor geldige vlucht
CSV_FILENAME        = "vlucht.csv"

# ── NeoPixel setup ──────────────────────────────────────────────────────────
neo = neopixel.NeoPixel(machine.Pin(NEO_PIN), 1)

def led(r, g, b):
    neo[0] = (r, g, b)
    neo.write()

def led_off():
    led(0, 0, 0)

# ── BME280 driver (minimaal, geen externe lib nodig) ────────────────────────
BME280_ADDR = 0x76

class BME280:
    def __init__(self, i2c):
        self.i2c = i2c
        self._load_calibration()
        # forced mode config: osrs_t=1, osrs_p=1, mode=0 (sleep first)
        self.i2c.writeto_mem(BME280_ADDR, 0xF2, bytes([0x01]))  # osrs_h=1
        self.i2c.writeto_mem(BME280_ADDR, 0xF4, bytes([0x27]))  # osrs_t=1, osrs_p=1, normal mode

    def _read_reg(self, reg, n):
        return self.i2c.readfrom_mem(BME280_ADDR, reg, n)

    def _load_calibration(self):
        raw = self._read_reg(0x88, 24)
        self.dig_T1 = struct.unpack_from('<H', raw, 0)[0]
        self.dig_T2 = struct.unpack_from('<h', raw, 2)[0]
        self.dig_T3 = struct.unpack_from('<h', raw, 4)[0]
        self.dig_P1 = struct.unpack_from('<H', raw, 6)[0]
        self.dig_P2 = struct.unpack_from('<h', raw, 8)[0]
        self.dig_P3 = struct.unpack_from('<h', raw, 10)[0]
        self.dig_P4 = struct.unpack_from('<h', raw, 12)[0]
        self.dig_P5 = struct.unpack_from('<h', raw, 14)[0]
        self.dig_P6 = struct.unpack_from('<h', raw, 16)[0]
        self.dig_P7 = struct.unpack_from('<h', raw, 18)[0]
        self.dig_P8 = struct.unpack_from('<h', raw, 20)[0]
        self.dig_P9 = struct.unpack_from('<h', raw, 22)[0]
        h1 = self._read_reg(0xA1, 1)[0]
        raw2 = self._read_reg(0xE1, 7)
        self.dig_H1 = h1
        self.dig_H2 = struct.unpack_from('<h', raw2, 0)[0]
        self.dig_H3 = raw2[2]
        self.dig_H4 = (raw2[3] << 4) | (raw2[4] & 0x0F)
        self.dig_H5 = (raw2[5] << 4) | (raw2[4] >> 4)
        self.dig_H6 = struct.unpack_from('<b', raw2, 6)[0]

    def read(self):
        """Leest temperatuur (°C) en druk (hPa). Geeft (temp, pressure)."""
        data = self._read_reg(0xF7, 8)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        # Temperatuur compensatie
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        t_fine = var1 + var2
        temp = t_fine / 5120.0

        # Druk compensatie
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return temp, 0.0
        pressure = 1048576.0 - adc_P
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = self.dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self.dig_P7) / 16.0
        pressure /= 100.0  # Pa -> hPa

        return temp, pressure


def pressure_to_altitude(pressure_hpa, baseline_hpa):
    """Hoogte tov baseline in meters via barometrische formule."""
    return 44330.0 * (1.0 - (pressure_hpa / baseline_hpa) ** (1.0 / 5.255))


# ── Hulpfuncties LED animaties ───────────────────────────────────────────────
def pulse_blue(n=5):
    """Blauwe pulserende animatie (kalibreren)."""
    for _ in range(n):
        for i in range(0, 40, 4):
            led(0, 0, i)
            time.sleep_ms(15)
        for i in range(40, 0, -4):
            led(0, 0, i)
            time.sleep_ms(15)

def blink(r, g, b, times=3, on_ms=200, off_ms=200):
    for _ in range(times):
        led(r, g, b)
        time.sleep_ms(on_ms)
        led_off()
        time.sleep_ms(off_ms)


# ── Hoofd programma ──────────────────────────────────────────────────────────
def main():
    # Zet BME280 aan via power pin
    vcc = machine.Pin(BME_VCC_PIN, machine.Pin.OUT)
    vcc.value(1)
    time.sleep_ms(100)  # wacht op opstart sensor

    # I2C initialiseren
    i2c = machine.I2C(0, sda=machine.Pin(BME_SDA_PIN), scl=machine.Pin(BME_SCL_PIN), freq=400000)

    # BME280 initialiseren
    bme = BME280(i2c)
    time.sleep_ms(500)

    # ── 1. KALIBRATIE ────────────────────────────────────────────────────────
    # Blauw pulserend = kalibreren
    print("Kalibreren... (baseline meting)")
    baseline_pressures = []
    for i in range(CALIB_SAMPLES + CALIB_WARMUP):
        pulse_blue(1)
        _, p = bme.read()
        if i >= CALIB_WARMUP:  # eerste 10 negeren
            baseline_pressures.append(p)
        time.sleep_ms(SAMPLE_INTERVAL_MS)

    baseline_hpa = sum(baseline_pressures) / len(baseline_pressures)
    print(f"Baseline druk: {baseline_hpa:.2f} hPa")

    # ── 2. KLAAR VOOR LANCERING ──────────────────────────────────────────────
    # Geel knipperend = wacht op lancering
    print("Klaar voor lancering! Wachtend op start...")
    while True:
        blink(30, 30, 0, times=1, on_ms=300, off_ms=300)
        _, p = bme.read()
        alt = pressure_to_altitude(p, baseline_hpa)
        alt_history.append(alt)
        if len(alt_history) > ALT_SMOOTH_N:
            alt_history.pop(0)
        avg_alt = sum(alt_history) / len(alt_history)
        if avg_alt >= LAUNCH_THRESHOLD:
            break

    # ── 3. METEN (vlucht) ────────────────────────────────────────────────────
    # Groen = meten
    led(0, 40, 0)
    print("Lancering gedetecteerd! Logging gestart.")
    flight_data = []  # lijst van (tijd_ms, hoogte_m, temp_c, druk_hpa)
    start_time = time.ticks_ms()
    max_alt = 0.0
    landed = False

    while not landed:
        t0 = time.ticks_ms()
        _, p = bme.read()
        alt = pressure_to_altitude(p, baseline_hpa)
        elapsed = time.ticks_diff(time.ticks_ms(), start_time)
        flight_data.append((elapsed, round(alt, 2)))

        alt_history.append(alt)
        if len(alt_history) > ALT_SMOOTH_N:
            alt_history.pop(0)
        avg_alt = sum(alt_history) / len(alt_history)

        if len(flight_data) > MIN_FLIGHT_SAMPLES and avg_alt <= LAND_THRESHOLD:
            landed = True

        # Wacht tot volgende sample interval
        elapsed_loop = time.ticks_diff(time.ticks_ms(), t0)
        wait = SAMPLE_INTERVAL_MS - elapsed_loop
        if wait > 0:
            time.sleep_ms(wait)

    print(f"Vlucht beëindigd. Max hoogte: {max_alt:.1f} m, {len(flight_data)} samples.")

    # ── 4. CSV OPSLAAN ───────────────────────────────────────────────────────
    # Paars knipperend = opslaan
    print(f"Opslaan naar {CSV_FILENAME}...")
    for _ in range(3):
        led(40, 0, 40)
        time.sleep_ms(200)
        led_off()
        time.sleep_ms(200)

    with open(CSV_FILENAME, "w") as f:
        f.write("tijd_ms,hoogte_m\n")
        for row in flight_data:
            f.write(f"{row[0]},{row[1]}\n")

    print("CSV opgeslagen!")

    # ── 5. KLAAR ─────────────────────────────────────────────────────────────
    # Wit = klaar
    led(40, 40, 40)
    print("Gereed. Data staat in:", CSV_FILENAME)

    # Zet sensor uit
    vcc.value(0)


if __name__ == "__main__":
    main()

