## Nexys A7-100T constraints for trim_fpga_top
## clock, reset button, USB-UART bridge, and one status LED.

## ---------------- Clock (100 MHz onboard oscillator) -----------------
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L12P_T1_MRCC_35 Sch=clk100mhz
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { clk }];

## ---------------- Reset (CPU_RESETN pushbutton, active-low) ----------
set_property -dict { PACKAGE_PIN C12   IOSTANDARD LVCMOS33 } [get_ports { rst_n }]; #IO_L3P_T0_DQS_AD1P_15 Sch=cpu_resetn

## ---------------- USB-UART bridge -------------------------------------
## Board-schematic naming is from the bridge chip's point of view:
##   UART_TXD_IN  = bridge's TXD, driving INTO the FPGA  -> this is our RX
##   UART_RXD_OUT = bridge's RXD, driven OUT from the FPGA -> this is our TX
set_property -dict { PACKAGE_PIN C4    IOSTANDARD LVCMOS33 } [get_ports { rx_serial }]; #IO_L7P_T1_AD6P_35 Sch=uart_txd_in
set_property -dict { PACKAGE_PIN D4    IOSTANDARD LVCMOS33 } [get_ports { tx_serial }]; #IO_L11N_T1_SRCC_35 Sch=uart_rxd_out

## ---------------- Status LED: lights up once results are fully sent --
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 } [get_ports { done_led }]; #IO_L18P_T2_A24_15 Sch=led[0]
