# How to configure UART communication between STM32 and ESP32
This tutorial will show you how to configure a UART communication between STM32 board and ESP32 board. You have to take into account that in this example I will configure STM32 as the transmitter board and ESP32 as the receiver board, so **this configuration only works in this scenario**.

## Requirements
- STM32 board (In this example, I use STM32F411RE)
- ESP32 board (In this example, I use ESP32 38 pins)
- STM32CubeMX Software (https://www.st.com/en/development-tools/stm32cubemx.html#overview)

## STM32 board pinout configuration
We will use STM32CubeMX sofware in order to configure pins of STM32 board. By default UART2 is enable, but in this case I will enable UART6 and configure in **Single Wire (Half-Duplex)** mode. It's important that the **Baud Rate** is configured at 115200 bits/s in both boards.

![182590756-f9dc89ee-fae9-4c8b-99e9-f6bbefd62427](https://user-images.githubusercontent.com/40604222/182591703-20282fe4-e0dc-40d0-bc48-19e37128dcf2.png)

Now, in the main loop of the code (main.c) you have to add this code in order to send data. This will send the word "Hello" each 2 seconds through PC_6 pin (UART6).
```c
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_UART_Transmit(&huart6, "Hello", 6, 1000);
    HAL_Delay(2000);
    /* USER CODE BEGIN 3 */
  }
```

## ESP32 board configuration
You have to add this configuration to your ESP32 code. This code will configure pin 17 as a transmitter and pin 16 as a receiver.

```c++
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

void setup() {
    Serial.begin(115200);
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, 18, 19));
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));
}
```

Once this is configure, you have to add a code that constantly check if there is any data to receive.
```c++
void loop() {
    // Read data from UART.
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    length = uart_read_bytes(uart_num, data, length, 100);
    
    if (length > 0) {
        char * test = (char*) data;
        Serial.println(test);
    }
}
```
## Circuit diagram
Connect a single cable through which the STM32 board will transmit the data (Pin PC_6) and the ESP32 will receive it (Pin IO16). Check out the pinout images of this repository.

![Untitled Sketch_bb](https://user-images.githubusercontent.com/40604222/182598429-a4fab4f5-53d2-487c-a12c-c043ab79718c.png)
