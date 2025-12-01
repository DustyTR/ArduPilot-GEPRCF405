# ArduPilot-GEPRCF405
Ardupilot Firmware for GEPRC-F405 FC 

# Build

- ```rm -rf build && cp Tools/bootloaders/MatekF405-TE-bdshot_bl.bin ./Tools/bootloaders/GEPRC-F405_bl.bin```

- ```./waf configure --board GEPRC-F405```
  
- ```./waf copter```

> Waf: Leaving directory `/home/dusty/Documents/ardupilot/build/GEPRC-F405'
> 
> BUILD SUMMARY
> Build directory: /home/dusty/Documents/ardupilot/build/GEPRC-F405
> | Target     |     Text (B) | Data (B) | BSS (B) | Total Flash Used (B) | Free Flash (B)  | External Flash Used (B) |
> | - | - | - | - | - | - | - |
> | bin/arducopter  |  964296  | 2940  |  128236  |   **967236** |  15800 | Not Applicable |        
>
> 'copter' finished successfully (1m17.948s)

# Flashing the firmware

Change your directory to ```build/GEPRC-F405/bin```

- ```cd build/GEPRC-F405/bin```

You can install this tool by ```sudo apt-get -y install dfu-util``` or ```yay -S dfu-util``` after the instalation, plug your FC holding down the BOOT0 button then it should start in DFU mode.

- ```sudo dfu-util -a 0 -s 0x08010000:leave -D arducopter.bin```

## Firmware that you can pick

This [hwdef.dat](https://github.com/DustyTR/ArduPilot-GEPRCF405/blob/main/ardupilot/libraries/AP_HAL_ChibiOS/hwdef/GEPRC-F405/hwdef.dat) is optimized and configured specifically for my custom drone build, so you shouldn't use it directly. However, since this project includes the AK09911C magnetometer driver which ArduPilot hasn't added yet along with other adjustments that help ArduPilot run fully on this minimal FC, you're welcome to use it for inspiration :)

If you want to keep developing on this board I would recommend you to check out this FC's [betaflight version](https://github.com/betaflight/unified-targets/blob/master/configs/default/GEPR-GEPRCF405_BT_HD.config) for GPIO layout and other sensors configurations.

Reference Image of FC (Ignore the ugly solder joints): 

![alt text](https://github.com/DustyTR/ArduPilot-GEPRCF405/blob/main/20251201_174022.jpg "GEPRC-F405 FPV")

![alt text](https://github.com/DustyTR/ArduPilot-GEPRCF405/blob/main/20251201_174018.jpg "Project")

This reference project includes 

- TF-Mini Lidar sensor ToF [serial]
- GPS U-Blox M10N over [serial]
- AK09911C magnetometer (compass) over [I2C]
- Bluejay bidirectional esc communucation over [DSHOT300]
- Simple telemetry over [Mavlink2] *(RC not included not in budget, thats why I customized this firmware 4 ardupilot)*

| Memory Section |	Start Address (Hex) |	Size |	Purpose |
| - | - | - | - |
| Bootloader | 0x08000000 |	64 KB (0x10000) |	Code to initialize the board, handle USB/DFU connections, and launch the main firmware. |
| Application Firmware | 0x08010000| 	~960 KB |	ArduPilot executable code (the main flight stack). This is the code that is being written when you flash a new firmware file. |
| Parameters/Storage |	Varies |	~16 KB |	Reserved section, often toward the end of Flash or dedicated external memory (DataFlash). |

GEPRC-F405 uses W25Q128 NOR memory on board so parameters/storage partition does store on memory via SPI.

Keep in mind that the F405 is a limited MCU, and this FC is tailored mainly for FPV. As a result, problems like unstable AHRS behavior or varying system-clock speeds can appear if DMA settings aren’t properly assigned or other CPU consumptive features are enabled.

