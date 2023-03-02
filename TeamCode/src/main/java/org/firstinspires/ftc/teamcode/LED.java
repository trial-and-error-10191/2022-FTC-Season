package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// This file is a subsystem file for the LED.
public class LED {
    DigitalChannel redLED, greenLED;

    boolean toggle = false;

    ElapsedTime waitTimeToggle = new ElapsedTime();

    public LED (HardwareMap hwMap) {

        redLED = hwMap.get(DigitalChannel.class, "redLED");
        greenLED = hwMap.get(DigitalChannel.class, "greenLED");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }
    
    public void teleLED (boolean toggleButton) {
        // If toggle button is pressed, set toggle to opposite value:
        if (toggleButton && waitTimeToggle.time() > .75) {
            toggle = !toggle;
            if (toggle) {

                redLED.setState(false);
                greenLED.setState(true);
            }

            else {

                redLED.setState(true);
                greenLED.setState(false);
            }
            waitTimeToggle.reset();
        }
    }

    public void autoLED (int colorNum) {
        // Numbers 1-3 have a specific color.
        
        if (colorNum == 1) {
            redLED.setState(true);
            greenLED.setState(false);
        }

        else if (colorNum == 2) {
            redLED.setState(false);
            greenLED.setState(true);
        }

        else if (colorNum == 3) {
            redLED.setState(false);
            greenLED.setState(false);
        }

    }
}
