package org.firstinspires.ftc.teamcode;


import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="IMUEncoder", group="Linear Opmode")
//@Disabled
public class IMUAndEncoder extends LinearOpMode {

    // Declare hardware:
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    ElapsedTime time = new ElapsedTime();

    private BNO055IMU imu = null;

    // Drive motor position variables:
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    IMU.Parameters pramIMU;

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;

    Orientation angels;

    float targetAngle;


    int clicks = 0;
    // Set constants:
    private double left;
    private double right;
    private double fast = 0.5; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerInch = 200; // empirically measured
    private double clicksPerDeg = 21.94; // empirically measured


    private double turnSpeed = 0;

    double leftFrontPower = .5;
    double leftBackPower = .5;
    double rightFrontPower = .5;
    double rightBackPower = .5;


    double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;

    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Configure hardware map:
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.


        // Set motor directions:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Let driver know encoders are reset:
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition());

        telemetry.update();


        // Wait for the game to start:
        waitForStart();

        angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        targetAngle = angels.firstAngle;
        time.reset();

        telemetry.addData("yaw:", " %f", angels.firstAngle);
        telemetry.addData("pitch", "%f", angels.secondAngle);
        telemetry.addData("role", "%f", angels.thirdAngle);
        telemetry.addData("angel:", "%f", angels.firstAngle);
        telemetry.addData("angle: ", "%f", angels.firstAngle);
        telemetry.update();

        while (opModeIsActive()) {

            // *****************Dead reckoning list*************
            // Distances in inches, angles in deg, speed 0.0 to 0.6

            moveForward(40, fast);




        }
    }







    private float Angle () {

        targetAngle = angels.firstAngle;
        float CurrentAngle;
        Orientation angels2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        CurrentAngle = angels2.firstAngle;

        float deltaAngle = targetAngle - CurrentAngle;

        return deltaAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .40;

        angle = Angle();

        if (angle == targetAngle) {
            correction = 0;
        }// no adjustment.
        else {
            correction = -angle;        // reverse sign of angle for correction.
        }
        correction = correction * gain;

        return correction;
    }
        private void moveForward(int howMuch, double speed ) {
        // "howMuch" is in inches. A negative howMuch moves backward.
            Angle();

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos += (int) (howMuch * clicksPerInch);
        lrPos += (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);



        // Set the drive motor run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(leftBackPower);
        leftBackMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);

           if(Angle() > targetAngle){
               leftFrontMotor.setPower(DRIVE_SPEED +.2);
               rightFrontMotor.setPower(DRIVE_SPEED);
               leftBackMotor.setPower(DRIVE_SPEED + .2);
               rightBackMotor.setPower(DRIVE_SPEED);
           }
           else if (Angle() < targetAngle){
               leftFrontMotor.setPower(DRIVE_SPEED);
               rightFrontMotor.setPower(DRIVE_SPEED + .2);
               leftBackMotor.setPower(DRIVE_SPEED);
               rightBackMotor.setPower(DRIVE_SPEED + .2);
           }
           else {
               leftFrontMotor.setPower(DRIVE_SPEED);
               rightFrontMotor.setPower(DRIVE_SPEED);
               leftBackMotor.setPower(DRIVE_SPEED);
               rightBackMotor.setPower(DRIVE_SPEED);
           }
            // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(leftBackPower);
        leftBackMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    private void turnClockwise(float whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    private void moveToLine(int howMuch, double speed) {
        // "howMuch" is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }
    private void correct(int howMuch,double WitchDirection) {

        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos += (int) (howMuch * clicksPerInch);
        lrPos += (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Move robot to new position:

        if (WitchDirection == left) {
            leftFrontMotor.setTargetPosition(-lfPos);
            rightFrontMotor.setTargetPosition(rfPos);
            leftBackMotor.setTargetPosition(-lrPos);
            rightBackMotor.setTargetPosition(rrPos);
        }
        if (WitchDirection == right) {
            leftFrontMotor.setTargetPosition(lfPos);
            rightFrontMotor.setTargetPosition(-rfPos);
            leftBackMotor.setTargetPosition(lrPos);
            rightBackMotor.setTargetPosition(-rrPos);
        }
    }

}




