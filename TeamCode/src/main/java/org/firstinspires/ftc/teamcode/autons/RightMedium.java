package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Vision;


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

        auto.encoderStrafe(strafeSpeed, 10, 1150, true, false, false);

        auto.encoderDrive(driveSpeed, -200, 10, false);

        auto.encoderDrive(driveSpeed, 1300, 10, false);

        goToPos(liftSpeed, 3, 10);


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

                openClaw();

                sleep(100);

                auto.encoderDrive(driveSpeed, -400, 10, false);

                goToPos(liftSpeed, 0, 10);

                auto.driveToEnd(strafeSpeed, pos, false, true);

                auto.encoderTurn(.50, -1600, 10);

            }
        }
    }
}