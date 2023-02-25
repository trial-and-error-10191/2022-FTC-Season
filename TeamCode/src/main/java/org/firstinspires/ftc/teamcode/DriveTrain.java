package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This file is a subsystem file for the drivetrain.
public class DriveTrain {

    // Hardware:
    DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    // Double variables:
    double maxDrivePower = 0.75;
    double precisionModeThrottle = 2;

    public DriveTrain(HardwareMap hwMap) {
        // Initializes motor names:
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hwMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBack");

        // Initializes motor directions:
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void teleDrive(double axial, double lateral, double yaw, boolean toggle) {

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double max;

        if (Math.abs(axial) > 0.05 || Math.abs(lateral) > 0.05 || Math.abs(yaw) > 0.05) {
            leftFrontPower =  axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;
        }

        // All code bellow this comment normalizes the values so no wheel power exceeds 100%:
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (leftFrontPower > maxDrivePower) {
            leftFrontPower = maxDrivePower;
        }
        if (leftBackPower > maxDrivePower) {
            leftBackPower = maxDrivePower;
        }
        if (rightFrontPower > maxDrivePower) {
            rightFrontPower = maxDrivePower;
        }
        if (rightBackPower > maxDrivePower) {
            rightBackPower = maxDrivePower;
        }
        if (leftFrontPower < - maxDrivePower) {
            leftFrontPower = - maxDrivePower;
        }
        if (leftBackPower < - maxDrivePower) {
            leftBackPower = - maxDrivePower;
        }
        if (rightFrontPower < - maxDrivePower) {
            rightFrontPower = - maxDrivePower;
        }
        if (rightBackPower < - maxDrivePower) {
            rightBackPower = - maxDrivePower;
        }

        // This is the code for the precision mode:
        if (toggle) {
            leftFrontDrive.setPower(leftFrontPower / precisionModeThrottle);
            rightFrontDrive.setPower(rightFrontPower / precisionModeThrottle);
            leftBackDrive.setPower(leftBackPower / precisionModeThrottle);
            rightBackDrive.setPower(rightBackPower / precisionModeThrottle);
        } else {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }

    }
}
