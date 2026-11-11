# limb board netlist

Generated from `electronics/boards.yaml` + `electronics/parts.yaml`.

## Components

| ref | part | description |
|---|---|---|
| U1 | ESP32S3 | ESP32-S3-DevKitC-1 (WROOM-1-N8) |
| U2 | SN65HVD230 | CAN transceiver 3.3 V |
| U3 | MCP3208 | 12-bit SPI ADC |
| U4 | MCP3208 | 12-bit SPI ADC |
| J1 | POWER_IN | power entry connector |
| J2 | JST_CAN | CAN connector (in/out) |
| J3 | JST_CAN | CAN connector (in/out) |
| JP1 | TERM_120 | 120R split termination (jumper) |
| OK1 | OPTO_ESTOP | e-stop sense optocoupler |
| R1 | RES_PULLUP | 10k pull-up |
| D1 | LED | status LED + 330R |
| C1 | CAP | ceramic capacitor |
| C2 | CAP | ceramic capacitor |
| C3 | CAP | ceramic capacitor |
| C4 | CAP | ceramic capacitor |
| ENC0 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| ENC1 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| ENC2 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| ENC3 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| ENC4 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| ENC5 | AS5048A_BOARD | 14-bit magnetic angle sensor (SPI) |
| PS0 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV0 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS1 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV1 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS2 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV2 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS3 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV3 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS4 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV4 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS5 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV5 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS6 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV6 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS7 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV7 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS8 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV8 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS9 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV9 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS10 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV10 | DIVIDER | resistor divider 10k/20k + 10nF |
| PS11 | PRESSURE_SENSOR | pressure transducer 0.5-4.5 V |
| DIV11 | DIVIDER | resistor divider 10k/20k + 10nF |
| SRV0 | SERVO | PWM servo |
| SRV1 | SERVO | PWM servo |
| SRV2 | SERVO | PWM servo |
| SRV3 | SERVO | PWM servo |
| SRV4 | SERVO | PWM servo |
| SRV5 | SERVO | PWM servo |
| SRV6 | SERVO | PWM servo |
| SRV7 | SERVO | PWM servo |

## Nets

| net | max V | connections |
|---|---|---|
| +3V3 | 3.3 | U1.3V3, U2.VCC, U3.VDD, U3.VREF, U4.VDD, U4.VREF, R1.A, C1.A, C2.A, C3.A, C4.A, ENC0.VDD3V, ENC1.VDD3V, ENC2.VDD3V, ENC3.VDD3V, ENC4.VDD3V, ENC5.VDD3V |
| +5V | 5.0 | J1.V5, U1.5V, PS0.VCC, PS1.VCC, PS2.VCC, PS3.VCC, PS4.VCC, PS5.VCC, PS6.VCC, PS7.VCC, PS8.VCC, PS9.VCC, PS10.VCC, PS11.VCC |
| ADC_CS0 |  | U1.GPIO10, U3.CS |
| ADC_CS1 |  | U1.GPIO9, U4.CS |
| CANH | 5.0 | U2.CANH, J2.CANH, J3.CANH, JP1.A |
| CANL | 5.0 | U2.CANL, J2.CANL, J3.CANL, JP1.B |
| CAN_5V | 5.0 | J2.V5, J3.V5 |
| CAN_RX | 3.3 | U1.GPIO5, U2.R |
| CAN_TX |  | U1.GPIO4, U2.D |
| ENC_CS0 |  | U1.GPIO14, ENC0.CSN |
| ENC_CS1 |  | U1.GPIO15, ENC1.CSN |
| ENC_CS2 |  | U1.GPIO16, ENC2.CSN |
| ENC_CS3 |  | U1.GPIO17, ENC3.CSN |
| ENC_CS4 |  | U1.GPIO18, ENC4.CSN |
| ENC_CS5 |  | U1.GPIO21, ENC5.CSN |
| ESTOP_SENSE | 3.3 | OK1.COLL, R1.B, U1.GPIO2 |
| GND | 0.0 | J1.GND, U1.GND, U2.GND, U2.RS, U3.AGND, U3.DGND, U4.AGND, U4.DGND, J2.GND, J3.GND, OK1.CATHODE, OK1.EMIT, D1.K, C1.B, C2.B, C3.B, C4.B, ENC0.GND, ENC1.GND, ENC2.GND, ENC3.GND, ENC4.GND, ENC5.GND, PS0.GND, DIV0.GND, PS1.GND, DIV1.GND, PS2.GND, DIV2.GND, PS3.GND, DIV3.GND, PS4.GND, DIV4.GND, PS5.GND, DIV5.GND, PS6.GND, DIV6.GND, PS7.GND, DIV7.GND, PS8.GND, DIV8.GND, PS9.GND, DIV9.GND, PS10.GND, DIV10.GND, PS11.GND, DIV11.GND, SRV0.GND, SRV1.GND, SRV2.GND, SRV3.GND, SRV4.GND, SRV5.GND, SRV6.GND, SRV7.GND |
| LED |  | U1.GPIO8, D1.A |
| PWM0 |  | U1.GPIO39, SRV0.SIG |
| PWM1 |  | U1.GPIO40, SRV1.SIG |
| PWM2 |  | U1.GPIO41, SRV2.SIG |
| PWM3 |  | U1.GPIO42, SRV3.SIG |
| PWM4 |  | U1.GPIO47, SRV4.SIG |
| PWM5 |  | U1.GPIO1, SRV5.SIG |
| PWM6 |  | U1.GPIO6, SRV6.SIG |
| PWM7 |  | U1.GPIO7, SRV7.SIG |
| P_ADC0 | 3.0001499999999997 | DIV0.OUT, U3.CH0 |
| P_ADC1 | 3.0001499999999997 | DIV1.OUT, U3.CH1 |
| P_ADC10 | 3.0001499999999997 | DIV10.OUT, U4.CH2 |
| P_ADC11 | 3.0001499999999997 | DIV11.OUT, U4.CH3 |
| P_ADC2 | 3.0001499999999997 | DIV2.OUT, U3.CH2 |
| P_ADC3 | 3.0001499999999997 | DIV3.OUT, U3.CH3 |
| P_ADC4 | 3.0001499999999997 | DIV4.OUT, U3.CH4 |
| P_ADC5 | 3.0001499999999997 | DIV5.OUT, U3.CH5 |
| P_ADC6 | 3.0001499999999997 | DIV6.OUT, U3.CH6 |
| P_ADC7 | 3.0001499999999997 | DIV7.OUT, U3.CH7 |
| P_ADC8 | 3.0001499999999997 | DIV8.OUT, U4.CH0 |
| P_ADC9 | 3.0001499999999997 | DIV9.OUT, U4.CH1 |
| P_RAW0 | 4.5 | PS0.OUT, DIV0.IN |
| P_RAW1 | 4.5 | PS1.OUT, DIV1.IN |
| P_RAW10 | 4.5 | PS10.OUT, DIV10.IN |
| P_RAW11 | 4.5 | PS11.OUT, DIV11.IN |
| P_RAW2 | 4.5 | PS2.OUT, DIV2.IN |
| P_RAW3 | 4.5 | PS3.OUT, DIV3.IN |
| P_RAW4 | 4.5 | PS4.OUT, DIV4.IN |
| P_RAW5 | 4.5 | PS5.OUT, DIV5.IN |
| P_RAW6 | 4.5 | PS6.OUT, DIV6.IN |
| P_RAW7 | 4.5 | PS7.OUT, DIV7.IN |
| P_RAW8 | 4.5 | PS8.OUT, DIV8.IN |
| P_RAW9 | 4.5 | PS9.OUT, DIV9.IN |
| SPI_MISO | 3.3 | U1.GPIO13, U3.DOUT, U4.DOUT, ENC0.MISO, ENC1.MISO, ENC2.MISO, ENC3.MISO, ENC4.MISO, ENC5.MISO |
| SPI_MOSI |  | U1.GPIO11, U3.DIN, U4.DIN, ENC0.MOSI, ENC1.MOSI, ENC2.MOSI, ENC3.MOSI, ENC4.MOSI, ENC5.MOSI |
| SPI_SCLK |  | U1.GPIO12, U3.CLK, U4.CLK, ENC0.CLK, ENC1.CLK, ENC2.CLK, ENC3.CLK, ENC4.CLK, ENC5.CLK |
| V_ACT_SENSE | 25.2 | J1.VBAT_SENSE, OK1.ANODE |
| V_SERVO | 7.4 | SRV5.V, SRV6.V, SRV7.V, J1.VSERVO |
| V_VALVE | 6.0 | SRV0.V, SRV1.V, SRV2.V, SRV3.V, SRV4.V, J1.VVALVE |

## MCU pin map

| function | GPIO |
|---|---|
| CAN_TX | 4 |
| CAN_RX | 5 |
| SPI_SCLK | 12 |
| SPI_MOSI | 11 |
| SPI_MISO | 13 |
| ENC_CS | [14, 15, 16, 17, 18, 21] |
| ADC_CS | [10, 9] |
| PWM | [39, 40, 41, 42, 47, 1, 6, 7] |
| ESTOP_SENSE | 2 |
| LED | 8 |
