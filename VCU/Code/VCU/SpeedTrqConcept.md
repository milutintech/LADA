### Code State
## 09.11.2023
> Testen mit ADAC 
> Testen Pcan View

### Torque and Speed controll  curve 

## Onroad 

# DMC

Läuft im drehmoment regulierten Modus.
0 - 655.34Nm und 0 - -655.36Nm	
16Bit wert .02Nm schritte
32767 - -32768

# ESP32

Kriegt 12 bit wert
0 - 4096

# Umrechnung

Input muss Umgewandelt und dann mit einem Vorwärts sig oder Rüchwärts sig  ge signed werden

also 12 bit wert in 16 bit signed int

beginnen mit map funktion


y = map(potiwert, 0, 4096, 0, 32767);
y = y | (fdbkSig << 15)