package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

// This file is a system file.
// Bessie is the name of our robot (thanks Jake).
public class Bessie {
    public DriveTrain driveTrain;
    public Gripper gripper;
    public LED led;
    public Lift lift;
    public Bessie(HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        gripper = new Gripper(hwMap);
        led = new LED(hwMap);
        lift = new Lift(hwMap);
    }

}
