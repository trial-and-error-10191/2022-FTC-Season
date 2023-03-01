package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// This file is the main OpMode file.
// Bessie is the name of the robot.
@TeleOp(name = "Energize Tele Op", group = "LinearOpMode")
public class EnergizeTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initiates Bessie!
        Bessie bessie = new Bessie(hardwareMap);
        bessie.gripper.autoGripper(false);

        waitForStart();
        while (opModeIsActive()) {

            // This controls the drive train:
            bessie.driveTrain.teleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, bessie.led.toggle);

            // This controls the grippers:
            bessie.gripper.teleGripper(gamepad2.right_bumper, gamepad2.left_bumper);

            // This controls the lift:
            bessie.lift.teleLift(-gamepad2.left_stick_y);

            // This controls the LED:
            bessie.led.teleLED(gamepad1.x);

            telemetry.addData("Front Driving Motors | Left, Right", "%4.2f, %4.2f",
                    bessie.driveTrain.leftFrontDrive.getPower(),
                    bessie.driveTrain.rightFrontDrive.getPower());
            telemetry.addData("Back Driving Motors | Left, Right", "%4.2f, %4.2f",
                    bessie.driveTrain.leftBackDrive.getPower(),
                    bessie.driveTrain.rightBackDrive.getPower());
            telemetry.addData("Lift Motors | Left, Right", "%4.2f, %4.2f",
                    bessie.lift.liftMotor1.getPower(),
                    bessie.lift.liftMotor2.getPower());
            telemetry.addData("Top Limit Switch Status",
                    String.valueOf(bessie.lift.limitSwitch.getState()));
            telemetry.addData("Bottom Limit Switch Status",
                    String.valueOf(bessie.lift.sensorTouch.getState()));
            telemetry.addData("Precision Mode Toggle",
                    String.valueOf(bessie.led.toggle));
            telemetry.addData("Precision Mode Increment",
                    bessie.driveTrain.increment);
            telemetry.update();
        }
    }
}
