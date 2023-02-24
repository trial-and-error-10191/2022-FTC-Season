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
        bessie.liftAndGripper.autoGripper(false);

        waitForStart();
        while (opModeIsActive()) {

            // This controls the drive train:
            bessie.driveTrain.teleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_stick_y);

            // This controls the grippers:
            bessie.liftAndGripper.teleGripper(gamepad2.right_bumper, gamepad2.left_bumper);

            // This controls the lift:
            bessie.liftAndGripper.teleLift(-gamepad2.left_stick_y);

            // This controls the LED:
            bessie.led.teleLED(gamepad1.x);
        }
    }
}
