package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

// This file is a system file.
// Bessie is the name of our robot (thanks Jake).
public class Bessie {
    public DriveTrain driveTrain;
    public LiftAndGripper liftAndGripper;
    public LED led;
    public Bessie(HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        liftAndGripper = new LiftAndGripper(hwMap);
        led = new LED(hwMap);
    }

}
