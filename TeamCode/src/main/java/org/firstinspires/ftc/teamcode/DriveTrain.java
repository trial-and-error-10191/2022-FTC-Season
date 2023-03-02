package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// This file is a subsystem file for the drivetrain.
public class DriveTrain {

    // Hardware:
    DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    // Double variables:
    double precisionModeThrottle = 2;
    double increment = 0.05;
    ElapsedTime waitTimeToggle = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     clicksPerInch         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double fast = 0.6; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerDeg = clicksPerInch / 4.99; // empirically measured

    private int lfPos, rfPos, lrPos, rrPos;

    private BNO055IMU       imu         = null;

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

    public void precision(boolean input) {
        if (input) {
            if (input && waitTimeToggle.time() > .75) {
                precisionModeThrottle += increment;
                waitTimeToggle.reset();
            }
        }
    }


}
