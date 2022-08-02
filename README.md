# fli3d_esp32

## Compilation instructions
To compile in the Arduino IDE, for ESP32 core v1.0.6, board "MH ET LIVE ESP32MiniKit" and with partition scheme "Default with spiffs" or custom partition scheme "Fli3d ESP32 (OTA/maximized SPIFFS)" provided.

To add this partition scheme, copy `fli3d-esp32.csv` to `C:\Users\[YOURNAME]\Documents\ArduinoData\packages\esp32\hardware\esp32\1.0.6\tools\partitions`, and add the following lines to `C:\Users\[YOURNAME]\Documents\ArduinoData\packages\esp32\hardware\esp32\1.0.6\boards.txt`:

`mhetesp32minikit.menu.PartitionScheme.huge_app=Fli3d ESP32 (OTA/maximized SPIFFS)`
`mhetesp32minikit.menu.PartitionScheme.huge_app.build.partitions=fli3d-esp32`
`mhetesp32minikit.menu.PartitionScheme.huge_app.upload.maximum_size=983040`

For the libraries needed to compile, see https://github.com/jmwislez/fli3d_lib

## Fli3d ESP32 physical configuration
For the physical configuration and connections, see https://github.com/jmwislez/fli3d

## Configuring
Customize the configuration files in "files_for_ftp", and upload them to the ESP32 file system through FTP (passive).
