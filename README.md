# ESP32 Light Controller

This project is a custom LED light controller using an ESP32, built to overcome certain limitations in typical market LED controllers, such as low frame rates and visual artifacts like strobing. It is meant to be used with lights that use the WS2811/2812 protocol.

## Features

- **High Frame Rate Support**: Capable of achieving up to **200 frames per second (fps)**, significantly higher than many market controllers that are limited to around 40 fps.
- **Customizable Sequences**: Works with lighting design software like **Vixen** and **Xlights** to create and export `.fseq` files.
- **Interfacing With SD Card**: Uses SDSPI library to take LED data off of an SD card, process it, and transmit it.
- **Efficient LED Control**: Leverages the ESP32's **RMT subsystem** to efficiently transmit processed data to LEDs.

## Challenges

- **Memory Constraints**: The typical LED light show is around **20MB uncompressed**, while the ESP32 has only about **4MB of flash memory**. This makes it impossible to store the entire show in memory.
- **Performance Bottlenecks**: Achieving the target of **200 fps** is challenging due to the time it takes to read data from the **SD card**.
- **Rebooting for New Shows**: Each time a new show is loaded, the ISR and timing components of the controller require a reboot to ensure the correct frame rate configuration is applied.

## How It Works

1. **Create Sequences**: `.fseq` files are created using **Vixen** or **Xlights**.
2. **Upload to SD Card**: The sequence is exported and saved to an SD card.
3. **ESP32 Processing**: 
   - The program parses the `.fseq` file header to determine the desired frame rate, as well as other specifications such as channel number and show size.
   - Tasks and ISRs are initialized with the correct configuration.
4. **Data Transmission**: 
   - Each frame's data is read from the SD card, processed, and transmitted to the LEDs using the ESP32's **RMT subsystem**.

## Build Process

1. Clone this repository
2. Set up the ESP32 development environment:
    - Follow the [ESP32 setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for your operating system.
3. Modify the code to set up the correct output/input pins for your ESP32 configuration.
4. Compile and flash the code to your ESP32:
    ```bash
    idf.py build
    idf.py flash
    ```
5. Program a show (in Vixen of XLights) and export the `.fseq` file (uncompressed) to an SD card. The framerate you export at is stored in the `.fseq` file.
6. Plug the lights into the ESP32 with the correct data, Vin, and ground pins. You will need a 5V power supply for WS2812 lights and a 12V power supply for WS2811 lights.
