package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.Vision;


/*
    Pos List:
    0 - Ground, picking up cone
    1 - Lifting cone off ground and before picking
    2 - Low
    3 - Medium
    4 - High
 */

@Autonomous(name="Left High", group="OnBot")

public class LeftHigh extends LinearOpMode {
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

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while(!isStarted()){
            pos = camera.getPos();
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        auto.resetEncoders();

        auto.lift.closeClaw();

        sleep(1000);

        auto.lift.goToPos(liftSpeed, 1, 10);

        auto.encoderStrafe(strafeSpeed, 10, 1150, false, false, false);

        auto.encoderDrive(driveSpeed, -200, 10, false);

        auto.encoderDrive(driveSpeed,1275, 10, false);

        auto.lift.goToPos(liftSpeed, 4, 10);

        boolean done = false;

        while(!done){
            done = auto.scanForPole(true);
            sleep(150);
        }

        if(done) {

            boolean isIn = false;

            while (!isIn) {
                isIn = auto.driveIntoPole();
            }

            if (isIn) {
                sleep(100);

                auto.lift.openClaw();

                sleep(100);

                auto.encoderDrive(driveSpeed, -300, 10, false);

                auto.lift.goToPos(liftSpeed, 0, 10);

                auto.driveToEnd(strafeSpeed, pos, true, false);

                auto.encoderTurn(.50, -1600, 10);
            }
        }
    }
}