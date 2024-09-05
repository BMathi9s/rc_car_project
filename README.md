# IEEE Smart and Radio Controlled Mobile Marshmallow Cannon

A smart, radio-controlled mobile platform featuring a marshmallow cannon powered by AI-driven auto-tracking and RF communication.

**Mathias Desrochers and the team**  
*Summer 2024*

---

## Usage

Run the AI: `python Marchemollw_Cannon/Auto_tracking_main.py`  
Test with Screen: `python Marchemollw_Cannon/Auto_tracking_with_screen.py`  
Flash RF Communication Code:  
- **RX_main/RX.ino** - Receiver (RC car)  
- **Tx_main/Tx.ino** - Transmitter (Remote control)  
Servo Calibration: `python Research_and_Development/servo_ctr_manual.py`

---

## Project Structure


`Marchemollw_Cannon` - AI for the marshmallow cannon:  
- **Auto_tracking_main.py** - Main AI code.  
- **Auto_tracking_with_screen.py** - For testing with a screen attached to the Raspberry Pi.  
- **Marshmellow_Cannon.py** - Cannon control for the servo motor.

`Remote_ESP32` - RF communication code for the RC car and remote:  
- **RX_main/RX.ino** - Receiver code for the RC car.  
- **Tx_main/Tx.ino** - Transmitter code for the remote control.
    - **Tx_main/TX_lib** - has all the library used
        - Look up the mpu6050 library to set the offset on the gyroscope

`Research and Development` - Contains research and test scripts:  
- **RnD_for_remote** - Remote control tests.  
- **RnD_stuff_implementation** - Tested auto-tracking implementations.  
- **RnD_stuff_models** - Various AI models tested.  
- **servo_ctr_manual.py** - Manual servo angle input for testing/calibration.  
- **test_camera.py** - Camera test script.
- **venv_command.txt** - Useful Linux commands.

`requirements.txt` - Dependencies for the Python virtual environment.
