import machine
import time
import bme280
import neopixel

# 1. SETUP HARDWARE
i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))
bme = bme280.BME280(i2c=i2c, address=0X76)

# NeoPixel op Pin 28 (pas aan indien nodig), 1 LED
led = neopixel.NeoPixel(machine.Pin(23), 1)

def set_color(r, g, b):
    led[0] = (r, g, b)
    led.write()

def get_druk():
    # De driver geeft (temp, druk, vochtigheid) terug
    data = bme.read_compensated_data()
    # Pak het tweede element (index 1) voor de druk
    druk_pascal = data[1] 
    # Omrekenen naar hPa
    return druk_pascal / 100.0

def bereken_hoogte_m(p, p0):
    return 44330 * (1 - (p / p0)**(1 / 5.255))

# 2. KALIBRATIE (ROOD)
set_color(255, 0, 0) # ROOD
print("Kalibreren...")
druk_lijst = []
for _ in range(50):
    druk_lijst.append(get_druk())
    time.sleep_ms(10)

# 3. KLAAR VOOR LANCERING (GROEN)
set_color(0, 255, 0) # GROEN
trigger_hoogte = 1.5 
print("Klaar voor lancering! Wachten op 2,5m stijging...")

while True:
    druk_nu = get_druk()
    hoogte_nu = bereken_hoogte_m(druk_nu, p0)
    if hoogte_nu >= trigger_hoogte:
        start_tijd_ms = time.ticks_ms()
        break
    time.sleep(0.01)

# 4. LOGGEN (BLAUW)
set_color(0, 0, 255) # BLAUW
buffer = []
max_seconden = 30
interval_ms = 50 
stilstand_teller = 0
vorige_hoogte_gemiddelde = 0

# Voor smoothing: bewaar de laatste 5 metingen
metingen_lijst = []

for i in range((max_seconden * 1000) // interval_ms):
    lus_start = time.ticks_ms()
    druk = get_druk()
    h_m = bereken_hoogte_m(druk, p0)
    h_cm = int(h_m * 100)
    tijd_relatief = (lus_start - start_tijd_ms) / 1000.0
    
    buffer.append(f"{i+1},{tijd_relatief:.2f},{h_m:.2f},{h_cm},{druk:.2f}")
    
    # --- NIEUWE SMOOTHING LOGICA ---
    metingen_lijst.append(h_cm)
    if len(metingen_lijst) > 5:
        metingen_lijst.pop(0)
    
    h_cm_gemiddelde = sum(metingen_lijst) / len(metingen_lijst)
    
    # Controleer of het gemiddelde nauwelijks verandert
    if abs(h_cm_gemiddelde - vorige_hoogte_gemiddelde) < 10: 
        stilstand_teller += 1
    else:
        stilstand_teller = 0
    
    vorige_hoogte_gemiddelde = h_cm_gemiddelde
    
    # 30 opeenvolgende metingen (ca. 1.5 sec) stilstand = landing
    if stilstand_teller > 30: 
        print("Landing gedetecteerd!")
        break
    # -------------------------------
    
    while time.ticks_diff(time.ticks_ms(), lus_start) < interval_ms:
        time.sleep_ms(1)

print("Klaar met loggen, nu opslaan...")

# 5. OPSLAAN (LED UIT / WIT KNIPPEREN)
set_color(255, 255, 255) # WIT tijdens schrijven
with open("vlucht_data.csv", "w") as f:
    f.write("meting_id,tijd_s,hoogte_m,hoogte_cm,luchtdruk_hPa\n")
    for regel in buffer:
        f.write(regel + "\n")

while True:
    set_color(0, 255, 0) # Knipper groen als hij veilig op de grond ligt
    time.sleep(0.5)
    set_color(0, 0, 0)
    time.sleep(0.5)


