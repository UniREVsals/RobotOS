

package org.firstinspires.ftc.teamcode.auton;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;


@Autonomous(name="UniREVsals Bot: B2", group="UniREVsals B2")
public class AprilTagAutonB2 extends LinearOpMode

{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 5;
    int MIDDLE = 15;
    int RIGHT = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive unibot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
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

        HardwareMecanum robot   = new HardwareMecanum();
        robot.init(hardwareMap);



















        ElapsedTime runtime = new ElapsedTime();
        //=========================================AutoMode START=========================================//
        waitForStart();
        runtime.reset();
        runtime.startTime();
        //======================================not case specific code====================================//
        Trajectory globalB2 =  unibot.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(90)
                .forward(100)
                .strafeRight(90)
                .forward(250)
                .strafeRight(90)
                .forward(50)
                //afinei preloaded kono
                .back(50)
                .strafeRight(90)
                .forward(50)
                .strafeRight(90)
                .forward(200)
                //pernei 2o kono
                .back(200)
                .strafeLeft(90)
                .back(50)
                .strafeLeft(90)
                .forward(50)
                //affinei 20 kono
                .build();
















        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory ---------------------------------ADD HERE--------------------------------------
        Trajectory park1 =unibot.trajectoryBuilder(new Pose2d(0,0,0))
                //park 1
                .back(50)
                .strafeRight(90)
                .forward(50)
                .build();

        unibot.followTrajectory(globalB2);
        if (runtime.seconds() < 10 || runtime.seconds() == 10){
            telemetry.addLine("add extra cone");
        }
        else{unibot.followTrajectory(park1);}
        }
        else if(tagOfInterest.id == MIDDLE){
            //trajectory ---------------------------------ADD HERE--------------------------------------
            Trajectory park2 =unibot.trajectoryBuilder(new Pose2d(0,0,0))
                    //park 2
                    .back(50)
                    .strafeRight(90)
                    .forward(50)
                    .strafeLeft(90)
                    .forward(100)
                    .build();
            unibot.followTrajectory(globalB2);
            if (runtime.seconds() < 10 || runtime.seconds() == 10){
                telemetry.addLine("add extra cone");
            }
            else {unibot.followTrajectory(park2);}
        }
        else if(tagOfInterest.id == RIGHT){
            //trajectory ---------------------------------ADD HERE--------------------------------------
            Trajectory park3 =unibot.trajectoryBuilder(new Pose2d(0,0,0))
                    //park3
                    .back(50)
                    .strafeRight(90)
                    .forward(50)
                    .strafeLeft(90)
                    .forward(200)
                    .build();

            unibot.followTrajectory(globalB2);
            if (runtime.seconds() < 10 || runtime.seconds() == 10){
                telemetry.addLine("add extra cone");}
            else {unibot.followTrajectory(park3);}
        }
        else{//!!!FAILSAFE!!!
        telemetry.addLine("If you see this something has gone very wrong");}




















        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    @SuppressLint("DefaultLocale")
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


    HardwareMecanum robot = new HardwareMecanum();
}