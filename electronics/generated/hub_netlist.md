# hub board netlist

Generated from `electronics/boards.yaml` + `electronics/parts.yaml`.

## Components

| ref | part | description |
|---|---|---|
| U1 | ESP32S3 | ESP32-S3-DevKitC-1 (WROOM-1-N8) |
| U2 | SN65HVD230 | CAN transceiver 3.3 V |
| U3 | MCP3208 | 12-bit SPI ADC |
| U4 | IMU_BNO085 | 9-axis IMU (I2C) |
| U5 | CURRENT_SENSOR | hall current sensor 100 A |
| J1 | POWER_IN | power entry connector |
| J2 | JST_CAN | CAN connector (in/out) |
| J3 | JST_CAN | CAN connector (in/out) |
| JP1 | TERM_120 | 120R split termination (jumper) |
| OK1 | OPTO_ESTOP | e-stop sense optocoupler |
| R1 | RES_PULLUP | 10k pull-up |
| R2 | RES_PULLUP | 10k pull-up |
| R3 | RES_PULLUP | 10k pull-up |
| R4 | RES_PULLUP | 10k pull-up |
| D1 | LED | status LED + 330R |
| ESC1 | ESC | BLDC ESC signal input |
| Q1 | MOSFET_LOWSIDE | low-side switch for dump valve / fan |
| Q2 | MOSFET_LOWSIDE | low-side switch for dump valve / fan |
| SW1 | LEVEL_SWITCH | reservoir float switch (to GND, MCU pull-up) |
| RT1 | NTC_DIVIDER | 10k NTC + 10k divider from 3V3 |
| RV1 | VBAT_DIVIDER | battery voltage divider 100k/12k |
| SRV0 | SERVO | PWM servo |
| SRV1 | SERVO | PWM servo |
| C1 | CAP | ceramic capacitor |
| C2 | CAP | ceramic capacitor |
| C3 | CAP | ceramic capacitor |
| C4 | CAP | ceramic capacitor |
| PS0 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV0 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS1 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV1 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS2 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV2 | DIVIDER | resistor divider 10k/20k + 10nF |

## Nets

| net | max V | connections |
|---|---|---|
| +3V3 | 3.3 | U1.3V3, U2.VCC, U3.VDD, U3.VREF, U4.VIN, U5.VCC, R1.A, R2.A, R3.A, R4.A, RT1.VTOP, C1.A, C2.A, C3.A, C4.A |
| +5V | 5.0 | J1.V5, U1.5V, PS0.VCC, PS1.VCC, PS2.VCC |
| ADC_CS0 |  | U1.GPIO10, U3.CS |
| CANH | 5.0 | U2.CANH, J2.CANH, J3.CANH, JP1.A |
| CANL | 5.0 | U2.CANL, J2.CANL, J3.CANL, JP1.B |
| CAN_5V | 5.0 | J2.V5, J3.V5 |
| CAN_RX | 3.3 | U1.GPIO5, U2.R |
| CAN_TX |  | U1.GPIO4, U2.D |
| DUMP_COIL | 25.2 | Q1.DRAIN |
| DUMP_GATE |  | U1.GPIO47, Q1.GATE |
| ESC_PWM |  | U1.GPIO21, ESC1.SIG |
| ESTOP_SENSE | 3.3 | OK1.COLL, R1.B, U1.GPIO17 |
| FAN_COIL | 25.2 | Q2.DRAIN |
| FAN_GATE |  | U1.GPIO18, Q2.GATE |
| GND | 0.0 | J1.GND, U1.GND, U2.GND, U2.RS, U3.AGND, U3.DGND, U4.GND, U5.GND, J2.GND, J3.GND, OK1.CATHODE, OK1.EMIT, D1.K, ESC1.GND, Q1.SOURCE, Q2.SOURCE, SW1.B, RT1.GND, RV1.GND, SRV0.GND, SRV1.GND, C1.B, C2.B, C3.B, C4.B, PS0.GND, DIV0.GND, PS1.GND, DIV1.GND, PS2.GND, DIV2.GND |
| I2C_SCL | 3.3 | U1.GPIO42, U4.SCL, R3.B |
| I2C_SDA | 3.3 | U1.GPIO41, U4.SDA, R2.B |
| IBAT_ADC | 3.3 | U5.OUT, U1.GPIO6 |
| IMU_INT | 3.3 | U1.GPIO40, U4.INT |
| IMU_RST |  | U1.GPIO39, U4.RST |
| LED |  | U1.GPIO8, D1.A |
| LEVEL_SW | 3.3 | SW1.A, R4.B, U1.GPIO16 |
| NECK_PWM0 |  | U1.GPIO14, SRV0.SIG |
| NECK_PWM1 |  | U1.GPIO15, SRV1.SIG |
| NTC_ADC | 3.3 | RT1.OUT, U1.GPIO7 |
| P_ADC0 | 3.0001499999999997 | DIV0.OUT, U3.CH0 |
| P_ADC1 | 3.0001499999999997 | DIV1.OUT, U3.CH1 |
| P_ADC2 | 3.0001499999999997 | DIV2.OUT, U3.CH2 |
| P_RAW0 | 4.5 | PS0.OUT, DIV0.IN |
| P_RAW1 | 4.5 | PS1.OUT, DIV1.IN |
| P_RAW2 | 4.5 | PS2.OUT, DIV2.IN |
| SPI_MISO | 3.3 | U1.GPIO13, U3.DOUT |
| SPI_MOSI |  | U1.GPIO11, U3.DIN |
| SPI_SCLK |  | U1.GPIO12, U3.CLK |
| VBAT | 25.2 | J1.VBAT_SENSE, RV1.IN, OK1.ANODE |
| VBAT_ADC | 2.37762 | RV1.OUT, U1.GPIO1 |
| V_SERVO | 7.4 | SRV0.V, SRV1.V, J1.VSERVO |

## MCU pin map

| function | GPIO |
|---|---|
| CAN_TX | 4 |
| CAN_RX | 5 |
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 13 |
| ADC_CS | [10] |
| I2C_SDA | 41 |
| I2C_SCL | 42 |
| IMU_INT | 40 |
| IMU_RST | 39 |
| ESC_PWM | 21 |
| DUMP_VALVE | 47 |
| FAN_PWM | 18 |
| NECK_PWM | [14, 15] |
| VBAT_ADC | 1 |
| IBAT_ADC | 6 |
| NTC_ADC | 7 |
| LEVEL_SW | 16 |
| ESTOP_SENSE | 17 |
| LED | 8 |
