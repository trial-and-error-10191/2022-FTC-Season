package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {

    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    private final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    private final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    private final double     clicksPerInch           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final double     clicksPerDeg            = clicksPerInch / 3.99; // empirically measured

    public Drivetrain(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm) {

        leftFrontMotor = lfm;
        rightFrontMotor = rfm;
        leftBackMotor = lbm;
        rightBackMotor = rbm;
    }

    public void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

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

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void moveWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos += (int) (24 * clicksPerInch);
            lrPos += (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos += (int) (-24 * clicksPerInch);
            lrPos += (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void moveHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos += (int) (12 * clicksPerInch);
            lrPos += (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos += (int) (-12 * clicksPerInch);
            lrPos += (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void strafe(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos -= (int) (howMuch * clicksPerInch);
        lrPos -= (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void strafeWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos -= (int) (24 * clicksPerInch);
            lrPos -= (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos -= (int) (-24 * clicksPerInch);
            lrPos -= (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void strafeHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos -= (int) (12 * clicksPerInch);
            lrPos -= (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos -= (int) (-12 * clicksPerInch);
            lrPos -= (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

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

    public void turnClockwise(int whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        int lfPos = leftFrontMotor.getCurrentPosition();
        int rfPos = rightFrontMotor.getCurrentPosition();
        int lrPos = leftBackMotor.getCurrentPosition();
        int rrPos = rightBackMotor.getCurrentPosition();

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


}
