**Introduction**

This Arduino-based multitool is designed for newer Toyota vehicles manufactured between 2015 and 2022, without Toyota ECU Security. The code supports various features that enhance vehicle functionality and development. It leverages the capabilities of the Arduino and a CAN Bus shield to interact with different vehicle buses, including the AVN Bus, SAFETY Bus, and ADAS Bus. It is crucial to enable only the relevant features based on the connected bus to avoid any potential damages.

Also this readme was partially auto generated because I am lazy.

This code is potentially dodgy depending on how you use it so please please please read this entire thing.

PRs welcome.

**Supported Features**

1. **Relay Output - Combination Meter Dimmer**

   Relay output based on the dimming state of the combination meter. This functionality is particularly useful in specific weather conditions where drivers need to turn on the vehicle's headlight without dimming the radio or head unit.

2. **Fake Parking Assist ECU**

   USE WITH CAUTION - Fake Parking Assist ECU, which allows the user to use additional hardware and software, like openpilot, to send IPAS (Intelligent Parking Assist System) steering commands to the EPS (Electric Power Steering) unit. THIS SHOULD NOT BE ENABLED UNLESS YOU KNOW WHAT YOU ARE DOING AS THIS WILL CAUSE IRREVESIBLE CONFIGURATION CHANGES TO YOUR VEHICLE'S ECUs. (ask me how I know)

3. **Automatic Door Locker**

   UNTESTED - Monitors specific conditions, such as doors being opened while the vehicle is in "D" gear and the vehicle speed crossing a threshold. When conditions are met, the doors automatically lock to enhance security. This is a new implementation of my existing code and I have not tested any of this, so expect issues.

4. **Hybrid Gear Converter**

   Useful in certain iDataLink Maestro setups. It assists when the Maestro module is unable to accurately determine a hybrid vehicle's gear. The converter ensures smooth gear communication between different vehicle components.

**Known Issues**

1. The door lock command is based on a debug command sent by Toyota Techstream, but the certification ECU sees it as you trying to lock the doors from the outside when the keys are inside. So the vehicle might briefly panic if you get out of the vehicle from the driver's door when IGN is ON.

2. Auto door lock only work on hybrids, if you are driving an ICE vehicle, everything you need to adapt the code to work on your platform is already included. Including gear values, CAN ID, and bit positions.

3. Sometimes, albeit rarely, the vehicle ignores our door commands, especially if the ECU is busy.

**Safety Precautions**

- **Bus Connection**: Ensure that the Arduino and CAN Bus shield are connected to the appropriate bus (AVN Bus, SAFETY Bus, or ADAS Bus). Enable features only if the Arduino is connected to the relevant bus to prevent potential damages.

- **Vehicle Compatibility**: This multitool is tailored for newer Toyota vehicles without Toyota ECU Security. Use it with compatible vehicles to achieve optimal results. Don't try to use this on anything older than TNGA (New MC based RAV4s are OK).

**Installation and Usage**

1. Connect the Arduino and CAN Bus shield to the desired vehicle bus (AVN, SAFETY, or ADAS).

2. Edit the configuration section of the code to enable or disable specific features based on the connected bus.

3. Upload the sketch to the Arduino using the Arduino IDE.

4. Observe the appropriate indicator (relay) behavior based on the dimming state of the combination meter.

5. Utilize the Fake Parking Assist ECU to send IPAS steering commands to the EPS for advanced driver assistance system development.

6. Experience the convenience of automatic door locking based on specified conditions.

7. Benefit from the hybrid gear converter functionality, especially in iDataLink Maestro setups.

**Important Notes**

- Always exercise caution when interfacing with vehicle systems. Comply with all applicable laws and regulations to ensure safe and legal operation.

- Thoroughly test the code and its functionalities in a controlled environment before deploying it in a real-world scenario.

- Be mindful of the Arduino and CAN Bus shield connection to the correct vehicle bus to avoid any potential damages. (Like powering your air conditioner with your laptop - also ask me how I know)

**Disclaimer**

This code is provided as-is and requires careful configuration based on specific hardware setups and vehicle compatibility. The developers of this code cannot be held responsible for any damages, issues, or legal implications arising from its use. Use this code responsibly and adhere to all safety and legal guidelines.
