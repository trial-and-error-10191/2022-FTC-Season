package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
}
