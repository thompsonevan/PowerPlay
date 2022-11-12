package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
        auto = new AutoCommon(this, red);


        Vision camera = new Vision();
        camera.start(hardwareMap);

        while (!isStarted()) {
            pos = camera.getPos();
        }

        auto.resetEncoders();

        auto.lift.closeClaw();

        auto.lift.goToPos(liftSpeed, 1, 10);

        auto.encoderDrive(driveSpeed, 250, 10, false);

        auto.lift.goToPos(liftSpeed, 2, 10);

        boolean done = false;

        while (!done) {
            done = auto.scanForPole(false);
        }

        if (done) {
            boolean isIn = false;

            while (!isIn) {
                isIn = auto.driveIntoPole();
            }

            if (isIn) {
                sleep(100);

                auto.lift.openClaw();

                sleep(100);

                auto.encoderDrive(driveSpeed, -200, 10, false);

                auto.lift.goToPos(liftSpeed, 0, 10);

                auto.driveToEnd(strafeSpeed, pos, false, true);
            }
        }
    }
}