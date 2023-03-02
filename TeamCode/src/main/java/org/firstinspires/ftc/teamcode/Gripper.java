package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// This file is a subsystem file for the lift and gripper.
public class Gripper {

    // Hardware:
    Servo leftServo, rightServo;

    // Servo Positions:
    double rightOpenPos = 1.0;
    double leftOpenPos = 0.0;
    double rightClosePos = 0.0;
    double leftClosePos = 1.0;

    public Gripper(HardwareMap hwMap) {
        // Servos:
        rightServo = hwMap.get(Servo.class,"rightservo");
        leftServo = hwMap.get(Servo.class,"leftservo");

    }

    public void autoGripper(boolean status) {
        // True status opens the grippers, false status closes the gripper:
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
}