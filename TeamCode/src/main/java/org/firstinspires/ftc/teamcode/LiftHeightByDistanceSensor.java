/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import androidx.appcompat.view.StandaloneActionMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

/**
 * {@link LiftHeightByDistanceSensor} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name = "Lift Heights via Distance Sensor", group = "Sensor")
public class LiftHeightByDistanceSensor extends LinearOpMode {

    private final ElapsedTime motorRampUpTime = new ElapsedTime();
    private DistanceSensor sensorRange;
    private DigitalChannel sensorTouch;

    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;

    private final double TRAPEZOID_START_POWER = 0.05;
    private final double TRAPEZOID_MAX_POWER = 0.75;
    // percentage of travel distance for ramping up to max power (trapezoid power profile)
    private final double RAMP_UP_LENGTH = 0.25;
    // percentage of travel distance for ramping down to stationary (trapezoid power profile)
    private final double RAMP_DOWN_LENGTH = 0.75;

    private final double LINEAR_MAX_POWER = 0.75;
    private final double LIFT_POWER_INCREMENT = 0.05;

    private final double TRIANGLE_START_POWER = 0.05;
    private final double TRIANGLE_MAX_POWER = 0.5;

    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;
    // target heights [cm]
    private final double LOW_HEIGHT = 10.0;
    private final double MED_HEIGHT = 20.0;
    private final double HIGH_HEIGHT = 30.0;
    private final double MAX_HEIGHT = 50.0;


    enum MotionProfile {
        LINEAR,
        TRAPEZOIDAL,
        TRIANGULAR
    }

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        liftMotor1  = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");

        // Motor directions should be such that positive power moves lift upwards
        // and negative power moves lift downwards.
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            if (gamepad2.y)
            {
                MoveLift(MotionProfile.TRAPEZOIDAL, HIGH_HEIGHT);
            }
            else if (gamepad2.x)
            {
                MoveLift(MotionProfile.TRAPEZOIDAL, MED_HEIGHT);
            }
            else if (gamepad2.b)
            {
                MoveLift(MotionProfile.TRAPEZOIDAL, LOW_HEIGHT);
            }

            // generic DistanceSensor methods.
            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
    }


    public void MoveLift(MotionProfile motorMotionProfile, double targetHeight) {

        double startingHeight = sensorRange.getDistance(DistanceUnit.CM);
        double travelDistance = Math.abs(startingHeight - targetHeight);
        double currentHeight = startingHeight;
        double travelDirection = Math.signum(startingHeight - targetHeight);
        double liftPower = 0.0;
        double distanceTraveled, percentage;
        motorRampUpTime.reset();

        while (currentHeight < targetHeight - WIGGLE_ROOM || currentHeight > targetHeight + WIGGLE_ROOM)
        {
            distanceTraveled = Math.abs(currentHeight - startingHeight);
            percentage = distanceTraveled / travelDistance;

            switch (motorMotionProfile) {
                case LINEAR:
                    liftPower = travelDirection * LinearPowerProfile(liftPower);
                    break;
                case TRAPEZOIDAL:
                    liftPower = travelDirection * TrapezoidalPowerProfile(percentage);
                    break;
                case TRIANGULAR:
                    liftPower = travelDirection * TriangularPowerProfile(percentage);
                    break;
            }
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

    // Use for 'long distance' travel
    public double TrapezoidalPowerProfile(double progress) {

        double outputPower = 0.0;
        if (motorRampUpTime.time() < 1.0)
        {
            outputPower = motorRampUpTime.time() * TRAPEZOID_MAX_POWER;
        } else if (progress > RAMP_DOWN_LENGTH)
        {
            double b = 4.0 * TRAPEZOID_MAX_POWER - 3.0 * TRAPEZOID_START_POWER;
            double m = 4.0 * (TRAPEZOID_START_POWER - TRAPEZOID_MAX_POWER);
            outputPower = m * progress + b;
        } else
        {
            outputPower = TRAPEZOID_MAX_POWER;
        }

        return outputPower;
    }

    public double LinearPowerProfile(double currentPower) {

        double outputPower = Math.abs(currentPower);
        if (outputPower < LINEAR_MAX_POWER)
        {
            outputPower += LIFT_POWER_INCREMENT;
        }
        return outputPower;
    }

    // Use for 'short distance' travel
    public double TriangularPowerProfile(double progress) {

        double outputPower, b, m;

        if (progress < 0.5)
        {
            b = TRIANGLE_START_POWER;
            m = 2.0 * (TRIANGLE_MAX_POWER - TRIANGLE_START_POWER);
        } else
        {
            b = 2.0 * TRIANGLE_MAX_POWER - TRIANGLE_START_POWER;
            m = 2.0 * (TRIANGLE_START_POWER - TRIANGLE_MAX_POWER);
        }

        outputPower = m * progress + b;
        return outputPower;
    }

    public void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public boolean NeedToStop () {

        return !sensorTouch.getState() || sensorRange.getDistance(DistanceUnit.CM) >= MAX_HEIGHT;
    }
}
