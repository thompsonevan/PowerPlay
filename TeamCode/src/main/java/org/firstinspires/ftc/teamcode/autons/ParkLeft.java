package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagAutonomousInitTest;
import org.firstinspires.ftc.teamcode.Robot;


/*
    Pos List:
    0 - Ground, picking up cone
    1 - Lifting cone off ground and before picking
    2 - Low
    3 - Medium
    4 - High
 */

@Autonomous(name="Park Left", group="OnBot")

public class ParkLeft extends LinearOpMode {
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
            if(pos == -41){
                pos = 1;
            }
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        closeClaw();

        goToPos(liftSpeed, 1, 10);

        AutoCommon.encoderStrafe(strafeSpeed, 10, 1500, false, false, false);

        AutoCommon.turnToAngleAuton(0);

        if(pos == 0) {
            AutoCommon.encoderDrive(driveSpeed, 1150, 10, false);
        } else if (pos == 2){
            AutoCommon.encoderDrive(driveSpeed, -1150, 10, false);
        }

        goToPos(liftSpeed, 0, 10);

//        AutoCommon.encoderTurn(.50, -1600, 10);

    }
}