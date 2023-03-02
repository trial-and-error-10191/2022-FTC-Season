package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// This file is a subsystem file for the lift and gripper.
public class Lift {

    // Hardware:
    DcMotor liftMotor1, liftMotor2;
    DistanceSensor sensorRange;
    DigitalChannel sensorTouch, limitSwitch;

    public Lift(HardwareMap hwMap) {
        // Lift motors:
        liftMotor1  = hwMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hwMap.get(DcMotor.class, "LiftMotor2");

        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Limit switches:
        limitSwitch = hwMap.get(DigitalChannel.class, "limitSwitch");
        sensorTouch = hwMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        // Distance sensor:
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
    }


    public void teleLift(double liftPower) {
        // If touch sensor or limit switch is pressed, stop power:
        if(!sensorTouch.getState() && liftPower < 0.0) {
            dualLift(0.0);
        }
        else if (!limitSwitch.getState() && liftPower > 0.0) {
            dualLift(0.0);
        }
        else {
            dualLift(liftPower);
        }
    }

    public void dualLift(double power) {
        // Uses one value to set power of both lift motors:
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }


}
