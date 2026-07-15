## ====================================================================
## SYSTEM CLOCK (100 MHz oscillator on Basys 3)
## ====================================================================
set_property PACKAGE_PIN W5 [get_ports clk] 
    set_property IOSTANDARD LVCMOS33 [get_ports clk]
    create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## ====================================================================
## PMOD HEADER JA - MOTOR DRIVER (DRV8833 CONTROLS)
## ====================================================================
set_property PACKAGE_PIN J1 [get_ports {in1}]
    set_property IOSTANDARD LVCMOS33 [get_ports {in1}]
set_property PACKAGE_PIN L2 [get_ports {in2}]
    set_property IOSTANDARD LVCMOS33 [get_ports {in2}]
set_property PACKAGE_PIN J2 [get_ports {in3}]
    set_property IOSTANDARD LVCMOS33 [get_ports {in3}]
set_property PACKAGE_PIN G2 [get_ports {in4}]
    set_property IOSTANDARD LVCMOS33 [get_ports {in4}]

## ====================================================================
## PMOD HEADER JB - IR LINE SENSORS (Map to s[3:0] bus)
## ====================================================================
# Mapping: s[3]=sensor1, s[2]=sensor2, s[1]=sensor3, s[0]=sensor4
set_property PACKAGE_PIN L17 [get_ports {s[3]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {s[3]}]
set_property PACKAGE_PIN M19 [get_ports {s[2]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {s[2]}]
set_property PACKAGE_PIN P17 [get_ports {s[1]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {s[1]}]
set_property PACKAGE_PIN R18 [get_ports {s[0]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {s[0]}]

## ====================================================================
## PMOD HEADER JC - ULTRASONIC SENSOR
## ====================================================================
set_property PACKAGE_PIN K17 [get_ports {trig}]
    set_property IOSTANDARD LVCMOS33 [get_ports {trig}]
set_property PACKAGE_PIN M18 [get_ports {echo}]
    set_property IOSTANDARD LVCMOS33 [get_ports {echo}]
set_property PACKAGE_PIN N17 [get_ports {JA_RX}]
    set_property IOSTANDARD LVCMOS33 [get_ports {JA_RX}]
## ====================================================================
## PMOD HEADER JB (BOTTOM ROW) - STEPPER ACTUATOR
## ====================================================================
set_property PACKAGE_PIN A15 [get_ports {stepper_a1}]
    set_property IOSTANDARD LVCMOS33 [get_ports {stepper_a1}]
set_property PACKAGE_PIN A17 [get_ports {stepper_a2}]
    set_property IOSTANDARD LVCMOS33 [get_ports {stepper_a2}]
set_property PACKAGE_PIN B16 [get_ports {stepper_b1}]
    set_property IOSTANDARD LVCMOS33 [get_ports {stepper_b1}]
set_property PACKAGE_PIN C15 [get_ports {stepper_b2}]
    set_property IOSTANDARD LVCMOS33 [get_ports {stepper_b2}]

## ====================================================================
## CONFIGURATION CONSTRAINTS
## ====================================================================
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIX4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

# LED outputs
set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19   IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19   IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19   IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18   IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15   IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14   IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14   IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
set_property -dict { PACKAGE_PIN V13   IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3    IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3    IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3    IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3    IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3    IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1    IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1    IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

## 