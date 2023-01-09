package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    private final Servo rightServo;
    private final Servo leftServo;

    private final double rightOpenPosition = 0.1;
    private final double rightClosedPosition = 0.97;
    private final double leftOpenPosition = 0.9;
    private final double leftClosedPosition = 0.03;

    public Gripper(Servo right, Servo left) {

        rightServo = right;
        leftServo = left;
    }

    public void Open() {
        rightServo.setPosition(rightOpenPosition);
        leftServo.setPosition(leftOpenPosition);
    }

    public void Close() {
        rightServo.setPosition(rightClosedPosition);
        leftServo.setPosition(leftClosedPosition);
    }
}
