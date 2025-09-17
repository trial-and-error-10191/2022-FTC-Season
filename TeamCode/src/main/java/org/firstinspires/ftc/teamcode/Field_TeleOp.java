package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "Field oriented controls", group = "LinearOpMode")
public class Field_TeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    private ElapsedTime waitTimeToggle = new ElapsedTime();
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private DigitalChannel limitSwitch = null;
    public IMU imu = null;

    //private DcMotor lift = null;
    Servo rightServo;
    Servo leftServo;
    double servoPosition = 0.0;

    private DigitalChannel sensorTouch;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    boolean toggle = false;

    double angles = 0;

    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        redLED = hardwareMap.get(DigitalChannel.class, "redLED");
        greenLED = hardwareMap.get(DigitalChannel.class, "greenLED");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        // Corresponds driving servo names to servo variables and set beginning position.
        rightServo = hardwareMap.get(Servo.class,"rightservo");
        rightServo.setPosition(servoPosition);
        leftServo = hardwareMap.get(Servo.class,"leftservo");
        leftServo.setPosition(1.0);

        // Corresponds lift motor names to motor variables.
        liftMotor1  = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        // Initializes motor directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor2.setDirection(DcMotor.Direction.FORWARD);

        //MoveLift(0.0);

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Waits for driver to press play.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();
        runtime.reset();

        double deadzone = 0.05;
        while (opModeIsActive()) {
            if (gamepad1.x && waitTimeToggle.time() > .75) {
                toggle = !toggle;
                if(toggle) {
                    redLED.setState(false);
                    greenLED.setState(true);

                }

                else{

                    redLED.setState(true);
                    greenLED.setState(false);

                }
                waitTimeToggle.reset();
            }
            // Opens claw.
            if (gamepad2.right_bumper) {
                rightServo.setPosition(1.0);
                leftServo.setPosition(0.0);
            }

            // Closes claw.
            if (gamepad2.left_bumper) {
                rightServo.setPosition(0.0);
                leftServo.setPosition(1.0);
            }
            if(!sensorTouch.getState() && -gamepad2.left_stick_y < 0.0) {
                dualLift(0.0);
            }
            else if (!limitSwitch.getState() && -gamepad2.left_stick_y > 0.0) {
                dualLift(0.0);
            }
            else {
                dualLift(-gamepad2.left_stick_y);
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            angles = -orientation.getYaw(AngleUnit.RADIANS);

            double fieldStrafe = gamepad1.left_stick_x;
            double fieldForward = -gamepad1.left_stick_y;
            double fieldTurn = gamepad1.right_stick_x;

            double robotForward = fieldForward * Math.cos(angles) + fieldStrafe * Math.sin(angles);
            double robotStrafe = fieldStrafe * Math.cos(angles) - fieldForward * Math.sin(angles);
            double robotTurn = fieldTurn;

            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double leftBackPower = 0;
            double rightBackPower = 0;

            if (Math.abs(robotForward) > deadzone || Math.abs(robotStrafe) > deadzone || Math.abs(robotTurn) > deadzone) {
                leftFrontPower = robotForward + robotStrafe + robotTurn;
                rightFrontPower = robotForward - robotStrafe - robotTurn;
                leftBackPower = robotForward - robotStrafe + robotTurn;
                rightBackPower = robotForward + robotStrafe - robotTurn;
            }

            double max;

            // All code below this comment normalizes the values so no wheel power exceeds 100%.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max; // leftFrontPower = leftFrontPower / max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            double sensitivity = 0.65;
            // The next four lines gives the calculated power to each motor.
            leftFrontDrive.setPower(leftFrontPower * sensitivity);
            rightFrontDrive.setPower(rightFrontPower * sensitivity);
            leftBackDrive.setPower(leftBackPower * sensitivity);
            rightBackDrive.setPower(rightBackPower * sensitivity);


            // Makes the limb arm extend/contract, and gives the option to have precise movement
            if (gamepad2.left_stick_y < 0.05 && gamepad2.left_stick_y > -0.05) {   // Makes sure there's no drifting
                gamepad2.left_stick_y = 0;
            }

            // Makes the limb arm rotate
            if (gamepad2.right_stick_x < 0.05 && gamepad2.right_stick_x > -0.05) { // Makes sure there's no drifting
                gamepad2.right_stick_x = 0;
            }
        }
    }
    public void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }
}

