# fli3d_esp32

## Compilation instructions
To compile in the Arduino IDE, with ESP32 core v1.0.6/v2.0.18/v3.0.x, board "MH ET LIVE ESP32MiniKit" and with partition scheme "Default with spiffs" or custom partition scheme "Fli3d ESP32 (OTA/maximized SPIFFS)" provided.

In ESP32 core v2.0.18 and v3.0.x, the custom partition will be loaded by default through the presence of the partions.csv file. To add the custom partition scheme in v1.0.6, copy `partitions/fli3d-esp32.csv` to `C:\Users\[YOURNAME]\Documents\ArduinoData\packages\esp32\hardware\esp32\1.0.6\tools\partitions`, and add the following lines to `C:\Users\[YOURNAME]\Documents\ArduinoData\packages\esp32\hardware\esp32\1.0.6\boards.txt`:

`mhetesp32minikit.menu.PartitionScheme.huge_app=Fli3d ESP32 (OTA/maximized SPIFFS)`
`mhetesp32minikit.menu.PartitionScheme.huge_app.build.partitions=fli3d-esp32`
`mhetesp32minikit.menu.PartitionScheme.huge_app.upload.maximum_size=983040`

For the libraries needed to compile, see https://github.com/jmwislez/fli3d_lib

## Fli3d ESP32 physical configuration
For the physical configuration and connections, see https://github.com/jmwislez/fli3d

## Configuring
Customize the configuration files in "files_for_ftp", and upload them to the ESP32 file system through FTP (passive).
