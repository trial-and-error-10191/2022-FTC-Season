package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftAndGripper {

    // Hardware:
    Servo leftServo, rightServo;
    DcMotor liftMotor1, liftMotor2;
    DistanceSensor sensorRange;
    DigitalChannel sensorTouch, limitSwitch;
    // Servo Positions:
    double rightOpenPos = 1.0;
    double leftOpenPos = 0.0;
    double rightClosePos = 0.0;
    double leftClosePos = 1.0;

    public LiftAndGripper(HardwareMap hwMap) {
        // Servos:
        rightServo = hwMap.get(Servo.class,"rightservo");
        leftServo = hwMap.get(Servo.class,"leftservo");

        // Lift motors:
        liftMotor1  = hwMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hwMap.get(DcMotor.class, "LiftMotor2");

        // Limit switches:
        limitSwitch = hwMap.get(DigitalChannel.class, "limitSwitch");
        sensorTouch = hwMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        // Distance sensor:
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
    }

    public void autoGripper(boolean status) {

        if (status) {
            rightServo.setPosition(rightOpenPos);
            leftServo.setPosition(leftOpenPos);
        }
        else {
            rightServo.setPosition(rightClosePos);
            leftServo.setPosition(leftClosePos);
        }

    }

    public void teleGripper(boolean openInput, boolean closeInput) {

        // Opens gripper.
        if (openInput) {
            rightServo.setPosition(rightOpenPos);
            leftServo.setPosition(leftOpenPos);
        }

        // Closes gripper.
        if (closeInput) {
            rightServo.setPosition(rightClosePos);
            leftServo.setPosition(leftClosePos);
        }

    }

    public void teleLift(double liftPower) {

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

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }


}
