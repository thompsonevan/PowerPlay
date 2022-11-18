package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagAutonomousInitTest;
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

@Autonomous(name="Evan's Right Cone Stack", group="OnBot")

public class ERightConeStack extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;
    double liftSpeed = 1;

    AprilTagAutonomousInitTest april = new AprilTagAutonomousInitTest();

    int pos = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        //auto = new AutoCommon(this, red);

        Robot.init(true, true, false, true, true, this);

        april.startVision(hardwareMap);

        while(!isStarted() && !isStopRequested()){
            april.runVision();
            pos = april.getPos();
            telemetry.addData("ID", pos);
            telemetry.update();
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        AutoCommon.resetEncoders();

        closeClaw();

        goToPos(liftSpeed, 1, 10);

        AutoCommon.encoderStrafe(strafeSpeed, 10, 1500, true, false, false);

        goToPos(liftSpeed, 2, 10);

        boolean done = false;

        while(!done){
            done = AutoCommon.scanForPole(false);
            sleep(150);
        }

        if(done) {

            AutoCommon.outerPassed = false;
            AutoCommon.isDone = false;


            boolean isIn = false;

            while (!isIn) {
                isIn = AutoCommon.driveIntoPole();
            }

            if (isIn) {
                AutoCommon.junctionPassed = false;
                AutoCommon.isIn = false;

                sleep(100);

                openClaw();

                sleep(100);

                AutoCommon.encoderDrive(driveSpeed, -300, 10, false);

                AutoCommon.encoderStrafe(strafeSpeed, 10, 500, true, false, false);

                AutoCommon.encoderDrive(driveSpeed, 900, 10, false);

                AutoCommon.turnToAngleAuton(0);

                AutoCommon.autoPickDropEnabled = true;

                AutoCommon.autoPickDropAuton(false);

                AutoCommon.encoderDrive(driveSpeed, -500, 10, false);

                AutoCommon.turnToAngleAuton(0);

                AutoCommon.encoderDrive(driveSpeed, -1850, 10, false);

                AutoCommon.encoderStrafe(strafeSpeed, 10, 350, false, false, false);

                goToPos(liftSpeed, 3, 10);

                done = false;

                while(!done){
                    done = AutoCommon.scanForPole(true);
                    sleep(150);
                }

                if(done) {

                    isIn = false;

                    while (!isIn) {
                        isIn = AutoCommon.driveIntoPole();
                    }

                    if (isIn) {
                        sleep(100);

                        openClaw();

                        sleep(100);

                        AutoCommon.encoderDrive(driveSpeed, -300, 10, false);

                        goToPos(liftSpeed, 0, 10);

                        AutoCommon.encoderStrafe(strafeSpeed, 10, 500, false, false, false);

                        AutoCommon.encoderDrive(driveSpeed, pos * (1200), 10, false);

                        goToPos(liftSpeed, 0, 10);
                    }
                }

            }
        }




//        AutoCommon.encoderTurn(.50, -1600, 10);

    }
}