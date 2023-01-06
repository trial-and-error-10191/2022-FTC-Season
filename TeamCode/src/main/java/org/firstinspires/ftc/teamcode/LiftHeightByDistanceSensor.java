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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private DistanceSensor sensorRange;

    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;

    //private DigitalChannel touchSensor = null;

    private final double MAX_POWER = 0.75;
    private final double LIFT_POWER_INCREMENT = 0.05;
    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;

    // target heights [cm]
    private final double LOW_HEIGHT = 10.0;
    private final double MED_HEIGHT = 20.0;
    private final double HIGH_HEIGHT = 30.0;

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

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            if (gamepad2.y)
            {
                MoveLift(HIGH_HEIGHT);
            }
            else if (gamepad2.x)
            {
                MoveLift(MED_HEIGHT);
            }
            else if (gamepad2.b)
            {
                MoveLift(LOW_HEIGHT);
            }

            // generic DistanceSensor methods.
            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
    }


    public void MoveLift(double targetHeight) {

        double liftPower = 0.0;
        while (sensorRange.getDistance(DistanceUnit.CM) < targetHeight - WIGGLE_ROOM)
        {
            liftPower = RampUpLiftPower(liftPower);
            dualLift(liftPower);
        }
        liftPower = 0.0;
        dualLift(liftPower);
        while (sensorRange.getDistance(DistanceUnit.CM) > targetHeight + WIGGLE_ROOM)
        {
            liftPower = RampDownLiftPower(liftPower);
            dualLift(liftPower);
        }
        liftPower = 0.0;
        dualLift(liftPower);
    }

    public double RampUpLiftPower(double liftPower) {

        if (liftPower < MAX_POWER)
        {
            return liftPower + LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public double RampDownLiftPower(double liftPower) {

        if (liftPower > - MAX_POWER)
        {
            return liftPower - LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }
}
