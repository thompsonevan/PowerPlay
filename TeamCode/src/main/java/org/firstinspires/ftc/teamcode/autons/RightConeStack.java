package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.closeClaw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.goToPos;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.openClaw;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.SensorsCommon;
import org.firstinspires.ftc.teamcode.vision.Vision;


/*
    Pos List:
    0 - Ground, picking up cone
    1 - Lifting cone off ground and before picking
    2 - Low
    3 - Medium
    4 - High
 */

@Autonomous(name="Right Cone Stack", group="OnBot")

public class RightConeStack extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .7;
    double strafeSpeed = .7;
    double liftSpeed = 1;

    int pos = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        //auto = new AutoCommon(this, red);

        Robot.init(true, true, false, true, true, this);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while (!isStarted()) {

            SensorsCommon.updateDistanceValues();
            pos = camera.getPos();
        }

        telemetry.addData("pos", pos);
        telemetry.update();

        AutoCommon.resetEncoders();

        closeClaw();

        //sleep(1000);

        goToPos(liftSpeed, 2, 10);

        AutoCommon.encoderStrafe(strafeSpeed, 10, 2000, true, false, false);

        AutoCommon.turnToAngleAuton(0);
        Thread.sleep(5000);

        AutoCommon.encoderStrafe(driveSpeed, 10, 570, true,false,false);

        AutoCommon.turnToAngleAuton(0);

        AutoCommon.encoderDrive(driveSpeed, 900, 10, false);

        AutoCommon.autoPickDropEnabled=true;

        AutoCommon.autoPickDropAuton(false);

        AutoCommon.encoderDrive(driveSpeed, -500, 10, false);

        AutoCommon.turnToAngleAuton(0);

        AutoCommon.encoderDrive(driveSpeed, -1125, 10, false);

        AutoCommon.turnToAngleAuton(0);

        AutoCommon.turnToAngleAuton(-90);

        Thread.sleep(5000);

        AutoCommon.turnToAngleAuton(0);

        AutoCommon.turnToAngleAuton(0);

        AutoCommon.encoderDrive(driveSpeed, 1125, 10, false);

        AutoCommon.autoPickDropEnabled=true;

        AutoCommon.autoPickDropAuton(false);
        // AutoCommon.autoDropConeOnJunction(5);


      //  goToPos(liftSpeed, 0, 10);

      //  AutoCommon.driveToEnd(strafeSpeed, pos, true, true);

      //  AutoCommon.encoderTurn(.50, -1600, 10);

    }
}