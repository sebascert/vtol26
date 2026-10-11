# SENSORES_INTEGRATION

## 1. SensorManager integration with CoordinadorDeTareas
The SensorManager is integrated with CoordinadorDeTareas to ensure timely management of sensor readings and task executions. This integration allows for effective scheduling of tasks based on sensor inputs, ensuring that the system responds dynamically to real-time data. The SensorManager collects data from the sensors and forwards it to the CoordinadorDeTareas, which then prioritizes and executes tasks based on predefined criteria.

## 2. Hardware connections
### I2C Connections
- Sensor 1: Address 0x68
- Sensor 2: Address 0x69

### SPI Connections
- Sensor 3: Chip Select (CS) pin connected to GPIO 10

### UART Connections
- Sensor 4: RX pin connected to GPIO 11
- Sensor 4: TX pin connected to GPIO 12

### ADC Pin Assignments
- Sensor 5: Pin A0
- Sensor 6: Pin A1

## 3. Sensor frequencies and task scheduling
- Sensor 1 Frequency: 10 Hz
- Sensor 2 Frequency: 5 Hz
- Sensor 3 Frequency: 20 Hz
- Sensor 4 Frequency: 1 Hz
- Task scheduling is based on the highest frequency sensor, with CoordinadorDeTareas triggering task executions accordingly.

## 4. Data flow diagram
![Data Flow Diagram](link-to-diagram)  
*Replace 'link-to-diagram' with the actual diagram URL.*

## 5. Calibration procedures
1. Power on the sensor and allow it to initialize.
2. Follow the manufacturer's calibration instructions.
3. Use known reference points to fine-tune the measurements.

## 6. Troubleshooting guide
### Problem: Sensor not responding
- Check hardware connections.
- Ensure the correct I2C/SPI/UART protocol is being used.

### Problem: Incorrect sensor readings
- Perform a calibration check.
- Verify environmental factors that may impact sensor performance.

### Problem: Data not being sent to CoordinadorDeTareas
- Look for communication errors and check task scheduling logic.

