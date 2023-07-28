
# Custom ESP32 Board with WS2812 LED Matrix

This project is designed for the ESP32 development framework (esp-idf) and is tailored to work with my custom ESP32 board. The board is equipped with 256 WS2812 LEDs, a buzzer, an LTR-303 light sensor, and an SHT3 temperature and humidity sensor. The main purpose of this project is to provide data visualization on the LED matrix and send the collected data to thingspeak.com for further visualization and analysis.

## Features

- Custom ESP32 board with 256 WS2812 LEDs for creating captivating visual effects.
- Built-in buzzer for sound output and notifications.
- LTR-303 light sensor for detecting ambient light levels.
- SHT3 temperature and humidity sensor for monitoring environmental conditions.
- Data visualization on the LED matrix, enabling dynamic and interactive displays.
- Sending collected data to thingspeak.com for visualization and analysis.

## Prerequisites

To use this project, you'll need the following:

- ESP32 development environment set up with esp-idf.
- The custom ESP32 board with WS2812 LEDs, buzzer, LTR-303, and SHT3 sensors.

## Installation

1. Clone this repository to your local machine.
2. Open the project in your esp-idf development environment.
3. Connect your custom ESP32 board to your computer.
4. Configure the project settings according to your board's specifications.
5. Build and flash the project to your ESP32 board.

## Usage

Upon successfully flashing the project to your custom ESP32 board, it will start visualizing data on the WS2812 LED matrix and collecting environmental data using the light and temperature/humidity sensors. The data will be sent to thingspeak.com for further analysis and visualization.

## Contributing

If you find any issues or have suggestions for improvement, feel free to open an issue or submit a pull request. Your contributions are welcome and appreciated!

## License

This project is licensed under the [MIT License](LICENSE). Feel free to use, modify, and distribute it according to the terms of the license.

Enjoy your custom ESP32 board with the mesmerizing LED matrix and data visualization capabilities! Happy coding!
