# CAN-Communication.
A small project where two microcontrollers communicate over CAN. The CAN messages contain values sampled against two different potentiometers and a switch that are connected to the transmitter MCU. These transmitted values are received by the receiver MCU over CAN and based on these received values the receiver MCU controls LEDs brightness level(instead of LEDs, something else like a motor speed, speaker volume level can also be controlled).
## Purpose of the project.
Understanding and implementing CAN communication between two microcontrollers. And based on the messages transmitted, the receiver MCU has to perform some task. The main purpose of the project is to develop a small system where the critical communication happens over CAN, and check the reliability of CAN communication.
## System Functionality.
### Transmitter side.
The transmitter MCU, which I will be calling the 'Driver MCU, has two potentiometers, and one switch. Both the potentiometers are connected to the same ADC, but different channels. Using this ADC, the values corresponding to the potentiometers are sampled and converted to a desired scale based on which the receiver MCU, which I will be calling the 'Driven MCU', will set the brightness of LEDs connected to it.

One of the potentiometers will be called 'Accelerator' and the other one will be called 'Brake'. 

The sampled values of accelerator will be distributed over a scale between 0 to 100. This means that at an instant the value of the accelerator will be between 0 to 100%.

Similarly the sampled value of brake will be distributed over a scale of 0 to 10. 

The switch connected to the driver MCU will be called 'Headlight'. The status of the switch i.e. ON/OFF will be transmitted to the driven MCU.

Apart from transmission of the values over CAN, these values will also be displayed over a serial terminal over UART communication. 
### Receiver side
The driven MCU is at the receiving end. All the sampled values and the status of the headlight switch is transmitted from the driver MCU and is received by the driven mcu. 
There are three LEDs connected to the driven MCU, each corresponding to accelerator, brake, and headlight switch at the driver MCU side. 

The brightness of the LED corresponding to the accelerator, will be at an instant between 1 to 100%, which is actually the value sampled at the accelerator pin in the driver MCU.

Similarly the brightness of the LED corresponding to brake will be at an instant between 0 to 10(0 to 100% brightness), which is the value sampled at the brake pin.

And based on the status ON/OFF of the headlight switch, the third LED is switched on and off.

The values received by the driven MCU will be displayed ove a serial terminal connected with the MCU over UART.

## CAN Analyzation.
Analyzation of the CAN message is done and visualised over PCAN View software.

## Product Image.

![WhatsApp Image 2024-07-08 at 11 42 27](https://github.com/Divyansh8757/CAN-Communication./assets/166917600/0e64f173-0c0c-46de-89ec-0f5f6353594e)

## Simulation Video.



https://github.com/Divyansh8757/CAN-Communication./assets/166917600/9b356215-f8fa-4c41-9b7f-4ccb5824a1ba


