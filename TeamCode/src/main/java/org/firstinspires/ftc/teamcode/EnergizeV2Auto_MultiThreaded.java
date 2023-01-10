package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Energize V2 Auto (MT)", group="Linear Opmode")
public class EnergizeV2Auto_MultiThreaded extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics.
    // UNITS ARE PIXELS.
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST1 = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 3; // Tag ID 3 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 9; // Tag ID 9 from the 36h11 family
    AprilTagDetection tagOfInterest = null;

    private final ElapsedTime waiting = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DistanceSensor sensorRange;
    private DigitalChannel sensorTouch;

    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;

    private Servo rightGripper;
    private Servo leftGripper;

    private final double fast = 0.6; // Fast speed
    private final double medium = 0.3; // Medium speed
    private final double slow = 0.1; // Slow speed

    // target heights [cm]
    private final double LOW_HEIGHT = 36.0;
    private final double MED_HEIGHT = 60.0;
    private final double HIGH_HEIGHT = 86.4;

    private Drivetrain drivetrain;
    private Gripper gripper;
    private Lift lift;

    // 1 = triangular
    // 2 = trapezoidal
    // anything else = linear
    // trapezoidal for long distances, triangular for short distances
    private int powerProfileChoice = 2;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        // Corresponds Drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        rightGripper = hardwareMap.get(Servo.class,"rightservo");
        leftGripper = hardwareMap.get(Servo.class,"leftservo");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        // Initializes Drive directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1  = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
        // Motor directions should be such that positive power moves lift upwards
        // and negative power moves lift downwards. In this case, both REVERSE
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        drivetrain = new Drivetrain(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        gripper = new Gripper(rightGripper, leftGripper);
        gripper.Open();
        lift = new Lift(liftMotor1, liftMotor2, sensorRange, sensorTouch, powerProfileChoice);
        lift.MoveLift(0.0);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST1 || tag.id == ID_TAG_OF_INTEREST2 ||tag.id == ID_TAG_OF_INTEREST3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // Generic autonomous plan
        drivetrain.moveWholeBlock("forward", fast);
        drivetrain.moveWholeBlock("forward", fast);
        drivetrain.moveWholeBlock("forward", medium);
        Wait(0.5);
        drivetrain.moveWholeBlock("backward", medium);
        Wait(0.5);
        drivetrain.turnClockwise(-90, slow);
        Wait(0.25);
        gripper.Close();
        drivetrain.turnClockwise(90, slow);
        Wait(0.25);
        lift.MoveLift(MED_HEIGHT);
        gripper.Open();
        lift.MoveLift(0.0);

        if (tagOfInterest != null) {
            if (tagOfInterest.id == 1) {

                drivetrain.strafeHalfBlock("forward", fast);
            } else if (tagOfInterest.id == 3) {

                drivetrain.strafeHalfBlock("backward", medium);
            } else if (tagOfInterest.id == 9) {

                drivetrain.moveHalfBlock("backward", slow);
            }
        } else { // if no tag was sighted, take a guess on which spot to park in

            drivetrain.moveHalfBlock("backward", slow);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void Wait(double waitTime) {

        waiting.reset();
        while (waiting.time() < waitTime) {

        }
    }
}