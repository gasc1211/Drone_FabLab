# Controller Design

The controller only has two ps2 joysticks that are used to send commands to the droneusing the nrf24l01 2.4ghz transiver.

## Controller Layout

### Left Joystick

- Up: Throttle up the motors (+)
- Down: Throttle down the motors (-)
- Left: Yaw left (-)
- Right: Yaw right (+)

### Right Joystick

- Up: Pitch forward (-)
- Down: Pitch backward (+)
- Left: Roll left (-)
- Right: Roll right (+)

![Pitch, Roll and Yaw Diagram](https://www.smlease.com/wp-content/uploads/2020/07/Roll-Yaw-Pitch.jpg)

## Inputs

- Each of the two Joystick positions (X,Y)
- Joystick buttons pressed

## Outputs

- Pitch, Roll and Yaw movements
- Motors throttle speed

## Joystick measurements

- width: 30mm
- height 34mm
- screwhole diameter: 3mm
- screwholes positions: 20mm width, 26 mm height
