package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Lift {

    private final DcMotor liftMotor1;
    private final DcMotor liftMotor2;

    private final DistanceSensor sensorRange;
    private final DigitalChannel sensorTouch;

    private final MotorPowerProfile powerProfile;

    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;

    // Lift should go no higher than this height to prevent structural damage
    private final double MAX_HEIGHT = 50.0;

    public Lift(DcMotor motor1, DcMotor motor2, DistanceSensor range, DigitalChannel touch, int profileChoice) {

        liftMotor1 = motor1;
        liftMotor2 = motor2;
        sensorRange = range;
        sensorTouch = touch;
        powerProfile = new MotorPowerProfile(profileChoice);
    }

    public void MoveLift(double targetHeight) {

        double startingHeight = sensorRange.getDistance(DistanceUnit.CM);
        double travelDistance = Math.abs(targetHeight - startingHeight);
        double currentHeight = startingHeight;
        double travelDirection = Math.signum(startingHeight - targetHeight);
        double liftPower = 0.0;
        double distanceTraveled, percentage;

        while (currentHeight < targetHeight - WIGGLE_ROOM || currentHeight > targetHeight + WIGGLE_ROOM)
        {
            distanceTraveled = Math.abs(currentHeight - startingHeight);
            percentage = distanceTraveled / travelDistance;

            liftPower = travelDirection * powerProfile.getPower(liftPower, percentage);
            dualLift(liftPower);

            if (NeedToStop())
            {
                break;
            }

            currentHeight = sensorRange.getDistance(DistanceUnit.CM);
        }
        liftPower = 0.0;
        dualLift(liftPower);
    }

    private void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    private boolean NeedToStop () {

        return !sensorTouch.getState() || sensorRange.getDistance(DistanceUnit.CM) >= MAX_HEIGHT;
    }
}

