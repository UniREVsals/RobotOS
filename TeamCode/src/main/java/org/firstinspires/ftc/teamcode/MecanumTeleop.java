package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="UniRevsals Bot: Teleop", group="UniRevsals bot")
//To Turn off teleop uncomment ---> @Disabled

//trajectory problem


public class MecanumTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();   // Use a Mecanum's hardware

    @Override
    public void runOpMode() {
        double x1 = 0.0; //    <--Left/Right (We need 0.0 to fix a bug)
        double y1 = 0.0; //    <--Front/Back (We need 0.0 to fix a bug)
        double liftPos = 0;
        double toSetPos = 0;

        //
        //  Fix  for 45 degree drift |
        //                           |
        //                           V
        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);

        double x2 = 0.0; //     <--(We need 0.0 to fix a bug)
        double y2 = 0.0;//      <--(We need 0.0 to fix a bug)
        double StaticPower = 3.0; //set the power and the time needed fot the 1 level displacement
        float StaticLevelTime = 1;//se seconds
        double ToExecPower = 0.0;


        //servo
        double armPos = HardwareMecanum.GRABER_HOME;
        double armSpeed = 0.01;





        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Awaiting Instructions");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        boolean GraberIsOpen = false;
        // run until the end of the match (driver presses STOP)
        while(opModeIsActive())  {
            double spin = gamepad1.right_stick_x; //For controlling the spin
            double OpenGraber = -1.0;
            double CloseGraber = 0.3;



            //===============================GRABER======================================//
            if(gamepad2.right_bumper){robot.Graber.setPosition(OpenGraber);}
            if(gamepad2.left_bumper){robot.Graber.setPosition(CloseGraber);}




            // rodes
            // if no one is pressing the right joystick move normally
            if (Math.abs(spin) > 0.1) {
                robot.FrontRightDrive.setPower(-spin);
                robot.BackRightDrive.setPower(-spin);
                robot.FrontLeftDrive.setPower(spin);
                robot.BackLeftDrive.setPower(spin);
                sleep(20);}
            else {
                // This way it's also easy to just drive straight, or just turn.
                y1 = -gamepad1.left_stick_y;
                x1 = gamepad1.left_stick_x;

                // needed for the 45 degree fix
                y2 = y1 * cosine45 + x1 * sine45;
                x2 = x1 * cosine45 - y1 * sine45;

                // Output the safe vales to the motor drives.
                robot.FrontLeftDrive.setPower(x2);
                robot.BackRightDrive.setPower(x2);
                robot.FrontRightDrive.setPower(y2);
                robot.BackLeftDrive.setPower(y2);
                sleep(20);}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++LIFT
//TODO:GLOBAL TEST LIFT CODE
            //pickup/ground junction (0)
            if (gamepad2.circle){
                if (liftPos != 0){
                    toSetPos = -liftPos;
                    liftPos = 0;
                    ToExecPower = (StaticPower * toSetPos);
                    robot.moveLift(ToExecPower,StaticLevelTime,(Telemetry)this);
                    sleep(20);}}
            //low junction (1)
            if (gamepad2.cross){
                if(liftPos != 1){
                    if(liftPos > 1){toSetPos = -1 * (liftPos - 1);}
                    else {toSetPos = (1);}
                    liftPos = 1;
                    ToExecPower = (StaticPower * toSetPos);
                    robot.moveLift(ToExecPower,StaticLevelTime,(Telemetry)this);
                    sleep(20);}}
            // med junction (2)
            if(gamepad2.square){
                if(liftPos == 3){
                    toSetPos = (-1);
                    liftPos = 2;}
                if(liftPos < 2){
                    toSetPos = (2 - liftPos);
                    liftPos = 2;}
                ToExecPower = (StaticPower * toSetPos);
                robot.moveLift(ToExecPower,StaticLevelTime,(Telemetry)this);
                sleep(20);}
            //high junctions (3)
            if(gamepad2.triangle){
                if(liftPos != 3){
                    toSetPos = (3 - liftPos);
                    liftPos = 3;
                    ToExecPower = (StaticPower * toSetPos);
                    robot.moveLift(ToExecPower,StaticLevelTime,(Telemetry)this);
                    sleep(20);}}

            //========================lift override==============================================//

            while (gamepad2.dpad_up){
                robot.UpLift(1);
                sleep(20);
            }
            while (gamepad2.dpad_down){
                robot.DownLift(1);
                sleep(20);
            }
































            //fix
            telemetry.update();
            sleep(50);
        }
    }
}
