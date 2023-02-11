package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOmniOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "strafe IMU")
public class StrafeIMU extends BasicOmniOpMode_Linear {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    BNO055IMU imu;
    Orientation angels;

    static final double speed = 0.5;
    float targetAngle;


    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();

        double leftFrontPower = -speed;
        double leftBackPower = speed;
        double rightFrontPower = speed;
        double rightBackPower = -speed;

        angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        targetAngle = angels.firstAngle;
        runtime.reset();

        rightFrontDrive.setPower(rightFrontPower);
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        while (opModeIsActive() && (runtime.seconds() < 5.0)) {


            angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);


            if (angels.firstAngle < targetAngle) {
                double yaw = checkDirection();

                leftFrontPower = -.3;
                leftBackPower = .3;
                rightFrontPower = -speed;
                rightBackPower = speed;


                rightFrontDrive.setPower(rightFrontPower);
                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);


                telemetry.addData("yaw:", " %f", angels.firstAngle);
                telemetry.addData("pitch", "%f", angels.secondAngle);
                telemetry.addData("role", "%f", angels.thirdAngle);
                telemetry.addData("angel:", "%f", angels.firstAngle);
                telemetry.addData("angle: ", "%f", angels.firstAngle);
                telemetry.update();



            }
            else if(angels.firstAngle > targetAngle){
                double yaw = checkDirection();

                leftFrontPower = -speed;
                leftBackPower = speed;
                rightFrontPower = .3;
                rightBackPower = -.3;

                rightFrontDrive.setPower(rightFrontPower);
                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);


                telemetry.addData("yaw:", " %f", angels.firstAngle);
                telemetry.addData("pitch", "%f", angels.secondAngle);
                telemetry.addData("role", "%f", angels.thirdAngle);
                telemetry.addData("angel:", "%f", angels.firstAngle);
                telemetry.addData("angle: ", "%f", angels.firstAngle);
                telemetry.update();



            }

            else{
                leftFrontPower = -speed;
                leftBackPower = speed;
                rightFrontPower = speed;
                rightBackPower = -speed;

                rightFrontDrive.setPower(rightFrontPower);
                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }
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

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .15;

        angle = Angle();

        if (angle == 0) {
            correction = 0;
        }// no adjustment.
        else {
            correction = -angle;        // reverse sign of angle for correction.
        }
        correction = correction * gain;

        return correction;
    }

}
