package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Vision;


/*
    Pos List:
    0 - Ground, picking up cone
    1 - Lifting cone off ground and before picking
    2 - Low
    3 - Medium
    4 - High
 */

@Autonomous(name="Left Medium", group="OnBot")

public class LeftMedium extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;
    double liftSpeed = 1;

    int pos = 2;

    @Override
    public void runOpMode() {
       
        Robot.init(true,true,false,true,true,this);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while(!isStarted()){
            pos = camera.getPos();
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        AutoCommon.resetEncoders();

        closeClaw();

        sleep(1000);

        goToPos(liftSpeed, 1, 10);

        AutoCommon.encoderStrafe(strafeSpeed, 10, 1150, false, false, false);

        AutoCommon.encoderDrive(driveSpeed, -200, 10, false);

        AutoCommon.encoderDrive(driveSpeed,1275, 10, false);

        goToPos(liftSpeed, 3, 10);

        boolean done = false;

        while(!done){
            done = AutoCommon.scanForPole(false);
            sleep(150);
        }

        if(done) {

            boolean isIn = false;

            while (!isIn) {
                isIn = AutoCommon.driveIntoPole();
            }

            if (isIn) {
                sleep(100);

                openClaw();

                sleep(100);

                AutoCommon.encoderDrive(driveSpeed, -300, 10, false);

                goToPos(liftSpeed, 0, 10);

                AutoCommon.driveToEnd(strafeSpeed, pos, false, false);

                AutoCommon.encoderTurn(.50, -1600, 10);
            }
        }
    }
}