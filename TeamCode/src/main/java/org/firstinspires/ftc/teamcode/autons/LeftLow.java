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

@Autonomous(name="Left Low", group="OnBot")

public class LeftLow extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;
    double liftSpeed = 1;

    int pos = 2;

    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        Robot.init(true,true,false,this);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while(!isStarted()){
            pos = camera.getPos();
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        auto.resetEncoders();

        closeClaw();

        sleep(1000);

        goToPos(liftSpeed, 1, 10);

        auto.encoderStrafe(strafeSpeed, 10, 1150, false, false, false);

        auto.encoderDrive(driveSpeed,250, 10, false);

        goToPos(liftSpeed, 2, 10);

        boolean done = false;

        while(!done){
            done = auto.scanForPole(false);
            sleep(150);
        }

        if(done) {

            boolean isIn = false;

            while (!isIn) {
                isIn = auto.driveIntoPole();
            }

            if (isIn) {
                sleep(100);

                openClaw();

                sleep(100);

                auto.encoderDrive(driveSpeed, -300, 10, false);

                goToPos(liftSpeed, 0, 10);

                auto.driveToEnd(strafeSpeed, pos, false, false);

                auto.encoderTurn(.50, -1625, 10);
            }
        }
    }
}