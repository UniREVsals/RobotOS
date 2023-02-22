package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class HardwareMecanum
{
    /* Public OpMode members. */
    public DcMotor  FrontLeftDrive   = null;
    public DcMotor  FrontRightDrive  = null;
    public DcMotor  BackLeftDrive    = null;
    public DcMotor  BackRightDrive   = null;
    public DcMotor  LiftLeft = null;
    public DcMotor  LiftRight = null;
    public Servo Graber = null;

    //Servo.cfg
    public final static double GRABER_HOME = 0.0;
    public final static double GRABER_MIN_RANGE = 0.0;
    public final static double GRABER_MAX_RANGE = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        FrontLeftDrive  = hwMap.get(DcMotor.class, "fl");
        FrontRightDrive = hwMap.get(DcMotor.class, "fr");
        BackLeftDrive = hwMap.get(DcMotor.class, "bl");
        BackRightDrive = hwMap.get(DcMotor.class, "br");
        LiftLeft = hwMap.get(DcMotor.class, "ll");
        LiftRight = hwMap.get(DcMotor.class, "rl");

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);// FORWARD or REVERSE
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);// FORWARD or REVERSE
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);// FORWARD or REVERSE
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);// FORWARD or REVERSE


        //Graber
        Graber = hwMap.servo.get("gb"); //servo graber
        Graber.setPosition(GRABER_HOME);//set the graber to 0

        FrontRightDrive.setPower(0.0);
        FrontLeftDrive.setPower(0.0);
        BackRightDrive.setPower(0.0);
        BackLeftDrive.setPower(0.0);
        LiftRight.setPower(0.0);
        LiftLeft.setPower(0.0);
    }
    public void moveLift(double power, float sec, Telemetry telemetry) {
        LiftLeft.setPower(power);
        LiftRight.setPower(power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < sec){
            telemetry.addData("Check", "LiftMove : runtime %2.5f S Elapsed" , runtime.seconds());
            telemetry.update();
        }
    }
    public void UpLift(double power) {
        LiftLeft.setPower(power);
        LiftRight.setPower(power);
    }
    public void DownLift(double power) {
        LiftLeft.setPower(-power);
        LiftRight.setPower(-power);
    }

    public void stopLift(){
        LiftRight.setPower(0);
        LiftLeft.setPower(0);
    }

}

