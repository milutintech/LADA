# Introduction 
During fermentation in wine production, heat is generated as a result of the chemical processes. To cool these fermentation tanks, glycol is pumped through cooling fins to control the tank temperature.
The development of this product was requested by Weingut Obrecht AG and carried out by MILUTIN.  



If you got any Questions, please contact the Repo-Owner.

## Repo-Owner
Luca  
 
# Getting Started 
Clone this Repo to your machine, now you are able to use all data from the Tankkühlung.  

# Device
The tank cooling system is characterized by the fact that it has 3 relay outputs, so for example a solenoid or servo valve can be controlled with 2 of the 3 relays. Additional functionality can be implemented using the 3rd relay. A Neopixel output is attached to the device for the display and an input for the temperature sensor is implemented.

"Insert  Image of Finished Product here!"

## Hardware
<img src="/Images/Image-Tankkuehlung.png" width="800">  

### Connectors
| Connector | Connector Style | Contact Cnt | Placed (Y/N) | Remark     |
| :---      | :----           | ----:       | :---:        | :--- |
| P10       | CTB9352/3       | 3           | Y            | Connector for NeoPixel (5V/GND)   |
| P11       | CTB9352/2       | 2           | Y            | Temperature Sensor Input (RTDIN+ / RTDIN-)      |
| P20       | 5-146130-2      | 6           | Y            | ESP-Serial, 5V, GND |
| P21       | 5-146130-2      | 6           | Y            | ESP-I2C, 3V3, 5V, GND |
| P22       | TBD             | 4           | N            | ESP-Serial, GND |
| P23       | TBD             | 10          | N            | ESP-Serial, GND |
| P30       | TBD             | 24          | N            | ESP-USB-OTG (Type C) |
| P40       | CTB9352/3       | 3           | Y            | 230VAC-IN (L, N, PE) |
| P41       | CTB9352/2       | 2           | Y            | 24VDC-OUT |
| P50       | CTB9352/3       | 3           | Y            | Connector Relay 1 |
| P51       | CTB9352/3       | 3           | Y            | Connector Relay 2 |
| P52       | CTB9352/3       | 3           | Y            | Connector Relay 3 |

### Pin-Mapping
| Signal | ESP32 HW Pin | ESP32 IO Pin | Used (Y/N) | Remark     |
| :---   | ----:        | :----        | :---:      |  :---      | 
| GND    | 1            | -            | Y | - |
| 3V3    | 2            | -            | Y | - |
| Enable | 3            | -            | Y | - |
| -      | 4            | IO4          | N | - |
| -      | 5            | IO5          | N | - |
| DOUT   | 6            | IO6          | Y | Data Output for NeoPixel |
| DRDY   | 7            | IO7          | Y |  |
| -      | 8            | IO15         | N | - |
| -      | 9            | IO16         | N | - |
| -      | 10           | IO17         | N | - |
| -      | 11           | IO18         | N | - |
| -      | 12           | IO8          | N | - |
| USB D- | 13           | IO19         | Y | USB OTG Device |
| USB D+ | 14           | IO20         | Y | USB OTG Device |
| -      | 15           | IO3          | N | - |
| -      | 16           | IO46         | N | - |
| RSTN   | 17           | IO9          | Y |  |
| SCSN   | 18           | IO10         | Y |  |
| MOSI   | 19           | IO11         | Y |  |
| SCLK   | 20           | IO12         | Y |  |
| MISO   | 21           | IO13         | Y |  |
| INTN   | 22           | IO14         | Y |  |
| K1     | 23           | IO21         | Y | Relay 1 control output |
| K2     | 24           | IO47         | Y | Relay 2 control output |
| LED1   | 25           | IO48         | Y | Programmable LED |
| -      | 26           | IO45         | N | - |
| BootOpt| 27           | IO0          | Y | Boot Option Button |
| -      | 28           | IO35         | N | - |
| K3     | 29           | IO36         | Y | Relay 3 control output |
| -      | 30           | IO37         | N | - |
| -      | 31           | IO38         | N | - |
| MTCK   | 32           | IO39         | Y |  |
| MTDO   | 33           | IO40         | Y |  |
| MTDI   | 34           | IO41         | Y |  |
| MTMS   | 35           | IO42         | Y |  |
| RX     | 36           | RXD0         | Y | Serial |
| TX     | 37           | TXD0         | Y | Serial |
| SCL    | 38           | IO2          | Y | I2C |
| SDA    | 39           | IO1          | Y | I2C |
| GND    | 40           | -            | Y | - |
| GND    | 41           | -            | Y | - |


## Software

<br>

# Build and Test
After a release new contributions are only allowed in new branches. After the Modifications have been checked by a secund Person, a new release will be created.  


# Contribute
Every body is welcome to commit to his repo and expand our Altium Components, Templates, etc. 

Keep in mind to only commit changes on a new branch, write notes and ask somebody else to review your changes.

If you just noticed a error and / or are unable to correct the mistake, please create an issue in the Github Repo. 



# To Do's: 
- [ ] ADD Product Image
- [ ] 


