package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

@Autonomous(name="Right Medium", group="OnBot")

public class RightMedium extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .3;
    double strafeSpeed = .5;
    double driveSpeedslow = .05;
    int drivedistance1block = 1000;
    int distancestrafe1block = 1200;
    double liftSpeed = 1;

    int pos;

    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while(!isStarted()){
            pos = camera.getPos();
        }




        auto.resetEncoders();

        auto.lift.closeClaw();

        auto.lift.goToPos(liftSpeed, 3, 10);

        auto.encoderDrive(driveSpeed, 100, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1150, true, false, false);

        auto.encoderDrive(driveSpeed, 1000, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 500, false, false, false);

        auto.encoderDrive(driveSpeed, 200, 10, false);

        auto.lift.goToPos(liftSpeed,2 , 10);

        auto.lift.openClaw();

        auto.encoderDrive(driveSpeed, -200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 500, true, false, false);

        auto.encoderDrive(driveSpeed, 1000, 10, false);

        auto.encoderTurn(.50, -730, 10);

        auto.driveToEnd(driveSpeed, pos);

    }
}