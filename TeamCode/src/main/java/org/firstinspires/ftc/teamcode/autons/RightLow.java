package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Vision;


@Autonomous(name="Right Low", group="OnBot")

public class RightLow extends LinearOpMode {
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
     
        Robot.init(true,true,false,true,true,this);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while (!isStarted()) {
            pos = camera.getPos();
        }

        AutoCommon.resetEncoders();

        closeClaw();

        goToPos(liftSpeed, 1, 10);

        AutoCommon.encoderDrive(driveSpeed, 250, 10, false);

        goToPos(liftSpeed, 2, 10);

        boolean done = false;

        while (!done) {
            done = AutoCommon.scanForPole(false);
        }

        if (done) {
            boolean isIn = false;

            while (!isIn) {
                isIn = AutoCommon.driveIntoPole();
            }

            if (isIn) {
                sleep(100);

                openClaw();

                sleep(100);

                AutoCommon.encoderDrive(driveSpeed, -200, 10, false);

                goToPos(liftSpeed, 0, 10);

                AutoCommon.driveToEnd(strafeSpeed, pos, false, true);
            }
        }
    }
}