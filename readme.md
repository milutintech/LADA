# LADA
## Interieur
### Mittelkonsole
Open Auto Pro

Waveshare display

PI 4

CPC200-CCPA (Car Play)

CSR8510 A10 (Bt

HIFI BERRY

https://bluewavestudio.io/community/thread-3044.html

### Tacho
Digifitz

## BMS
Orion 
### Ansteuerung
<img width="1110" alt="image" src="https://github.com/milutintech/LADA/assets/102026699/e5c6d471-01e4-414f-a05f-dc18713be604">

### Verkabelung
<img width="1037" alt="image" src="https://github.com/milutintech/LADA/assets/102026699/9ce96645-6f70-4e7d-9e8c-c9c29596b39d">

https://www.orionbms.com/manuals/pdf/orionbms2_wiring_manual.pdf

> ● If fewer than 12 cells are populated in a group, unused cell taps must all be connected to the highest
potential cell in that group. For example, if 6 cells are populated in a group, taps 6 – 12 all must be
connected to the positive tap on cell 6.*

> ● No cell group may have fewer than 4 cells connected (higher minimums may apply if a cell's
minimum voltage is below 2.8v). If a group of cells ends up with fewer than 4 cells, some cells must
be skipped in one of the other groups and wired into the group with less than 4 to ensure that a
minimum of 4 cells are present.

> ● Cell Groups must have a minimum normal working voltage of 11 volts (2.8v per cell for 4 cells) and
an absolute minimum voltage of 10v. Accuracy of voltage measurements is decreased when the
group voltage is below 11 volts.

> ● If a cell group has 0 cells connected (the group is skipped entirely), all wires from that group may be
left disconnected.

<img width="1215" alt="image" src="https://github.com/milutintech/LADA/assets/102026699/ad228a8d-6bfb-4327-bc98-67f4e28c8786">


### The BMS has two primary modes of operation: Charge Mode and Ready Mode. The BMS will enter into Charge Mode any time 12 - 24v is applied to the CHARGE power pin (Main I/O pin 3), regardlessof whether READY power is also present or not.

https://www.orionbms.com/manuals/pdf/orionbms2_operational_manual.pdf

> The following functions are enabled (or change) when the BMS is in Charge Mode:
> 1. The charger safety output is allowed to turn on if enabled and if all criteria have been met.
> 2. The BMS will cap the state of charge to the value specified as the "Charged SOC" percentage.
Even if the battery is charged in such a way that would normally cause the SOC to rise above
this value, the value will not exceed the "Charged SOC parameter" while the BMS is in charge
mode.

> 4. When any cell voltage hits the maximum cell voltage (resulting in the BMS turning the charger
off), the BMS will immediately adjust the state of charge to the "Charged SOC" value since the
BMS knows that the battery pack is fully charged at this time.

> 5. The cell balancing algorithm is enabled and will begin balancing as soon as any cell voltage
goes above the "Start Balancing" voltage. Balancing will continue until all cell voltages are
balanced to within the balance delta voltage parameter. See the “How Balancing Works” section
for more information on cell balancing.

> 6. Certain CANBUS messages may be transmitted or not transmitted depending on whether the
BMS is in CHARGE mode or not (if configured).

> 7. The maximum possible current limit for charging is limited to the "Maximum Amperage While
Charging" parameter available on the “Charge Limits” profile settings tab.
> 8. The maximum allowable cell voltage is limited to the "Max. Voltage While Charging" parameter
available on the “Cell Settings” profile settings tab.

### Flussdiagramme Charge und Discharge

charge

<img width="897" alt="image" src="https://github.com/milutintech/LADA/assets/102026699/df37af26-407f-4e66-90c3-341093233b45">

discharge

<img width="915" alt="image" src="https://github.com/milutintech/LADA/assets/102026699/64be99fb-769b-4ce5-839e-1b2a66a95da4">

