package org.firstinspires.ftc.teamcode.autons;

import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1.executeDrive;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1.getAngle;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.centerCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveLF;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveLR;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveRF;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveRR;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.junctionDriveDirCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.leftConeCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.rightConeCheck;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;


public class AutoCommon {
    

//    public DrivetrainCommon_ALT1 drivetrain;

    public VectorF blockLoc = null;
    public CameraDevice vufCam = null;

    private ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode curOpMode = null;

    double correction;

    public AutoCommon(LinearOpMode owningOpMode, boolean red) {


        curOpMode = owningOpMode;

        DrivetrainCommon_ALT1.initDrivetrainCommon_ALT1();
        LiftClawCommon.initLiftClawCommon();
//        drivetrain = new DrivetrainCommon_ALT1(curOpMode);

    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS, boolean pid) {
        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());
            }

            // Determine new target position, and pass to motor controller;
            driveLF.setTargetPosition(encoderValue);
            driveRF.setTargetPosition(encoderValue);
            driveLR.setTargetPosition(encoderValue);
            driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(speed));
            driveRF.setPower(Math.abs(speed));
            driveLR.setPower(Math.abs(speed));
            driveRR.setPower(Math.abs(speed));

            double currentPower = speed;

            int decelStart = (int) (encoderValue * .75);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (driveLF.isBusy() && driveRF.isBusy()
                            && driveLR.isBusy() && driveRR.isBusy()
                    )) {

                driveLF.setPower(Math.abs(currentPower - correction));
                driveRF.setPower(Math.abs(currentPower + correction));
                driveLR.setPower(Math.abs(currentPower - correction));
                driveRR.setPower(Math.abs(currentPower + correction));

                // currentPower=rampUpDown(speed,currentPower,.2,driveLF.getCurrentPosition(),decelStart);


                if (pid) {
                    correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());

                    if (correction > 0) {

                    }


                    if (encoderValue < 0) {
                        correction = correction * (-1);
                    }
                }
            }


            // Stop all motion;
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLR.setPower(0);
            driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderDriveToDistance(double speed,
                                       int encoderValue,
                                       double timeoutS, double distance, boolean pid) {
        double correction = 0;


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            if (pid) {
                correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());
            }

            // Determine new target position, and pass to motor controller;
            driveLF.setTargetPosition(encoderValue);
            driveRF.setTargetPosition(encoderValue);
            driveLR.setTargetPosition(encoderValue);
            driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(speed));
            driveRF.setPower(Math.abs(speed));
            driveLR.setPower(Math.abs(speed));
            driveRR.setPower(Math.abs(speed));

            double currentPower = speed;

            int decelStart = (int) (encoderValue * .75);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (driveLF.isBusy() && driveRF.isBusy()
                            && driveLR.isBusy() && driveRR.isBusy()
                    )) {


                driveLF.setPower(Math.abs(currentPower - correction));
                driveRF.setPower(Math.abs(currentPower + correction));
                driveLR.setPower(Math.abs(currentPower - correction));
                driveRR.setPower(Math.abs(currentPower + correction));

                // currentPower=rampUpDown(speed,currentPower,.2,driveLF.getCurrentPosition(),decelStart);
/*
                if(liftClaw.robot.lift_check.getDistance(DistanceUnit.CM)<distance)
                {
                    //curOpMode.sleep(500);
                    leftGuide.setPosition(.6);
                    rightGuide.setPosition(.4);
                    curOpMode.sleep(500);
                    break;
                }
*/
                if (pid) {
                    correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());

                    if (correction > 0) {

                    }


                    if (encoderValue < 0) {
                        correction = correction * (-1);
                    }
                }
            }


            // Stop all motion;
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLR.setPower(0);
            driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderDriveWithDrift(double leftSpeed, double rightSpeed,
                                      int encoderValue,
                                      double timeoutS) {


        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller;
            driveLF.setTargetPosition(encoderValue);
            driveRF.setTargetPosition(encoderValue);
            driveLR.setTargetPosition(encoderValue);
            driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(leftSpeed));
            driveRF.setPower(Math.abs(rightSpeed));
            driveLR.setPower(Math.abs(leftSpeed));
            driveRR.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (driveLF.isBusy() && driveRF.isBusy()
                            && driveLR.isBusy() && driveRR.isBusy()
                    )) {

                driveLF.setPower(Math.abs(leftSpeed));
                driveRF.setPower(Math.abs(rightSpeed));
                driveLR.setPower(Math.abs(leftSpeed));
                driveRR.setPower(Math.abs(rightSpeed));


            }


            // Stop all motion;
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLR.setPower(0);
            driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurn(double speed,
                            int encoderValue,
                            double timeoutS) {

        resetEncoders();
        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {


            double correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());

            // Determine new target position, and pass to motor controller;
            driveLF.setTargetPosition(-encoderValue);
            driveRF.setTargetPosition(encoderValue);
            driveLR.setTargetPosition(-encoderValue);
            driveRR.setTargetPosition(encoderValue);


            // Turn On RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            driveLF.setPower(Math.abs(speed));
            driveRF.setPower(Math.abs(speed));
            driveLR.setPower(Math.abs(speed));
            driveRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (driveLF.isBusy() && driveRF.isBusy()
                            && driveLR.isBusy() && driveRR.isBusy()
                    )) {

                driveLF.setPower(Math.abs(-speed));
                driveRF.setPower(Math.abs(speed));
                driveLR.setPower(Math.abs(-speed));
                driveRR.setPower(Math.abs(speed));


            }

            // Stop all motion;
            driveLF.setPower(0);
            driveRF.setPower(0);
            driveLR.setPower(0);
            driveRR.setPower(0);


            // Turn off RUN_TO_POSITION
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            curOpMode.sleep(500);   // optional pause after each move

            DrivetrainCommon_ALT1.rotation = getAngle();
            // reset angle tracking on new heading.
            DrivetrainCommon_ALT1.resetAngle();
        }
    }


    public void resetEncoders() {
        //Reset the encoders
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set all the motors to run using encoders
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public VectorF encoderStrafe(double power, double timeoutS, int encoderValue, boolean strafeLeft,
                                 boolean pid, boolean objectDetection) {

        DrivetrainCommon_ALT1.rotation = getAngle();        // reset angle tracking on new heading.
        DrivetrainCommon_ALT1.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        if(pid){
            correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());
        } else {
            correction = 0;
        }

        double currentPower = .1;

        runtime.reset();

        VectorF blockLoc = null;

        resetEncoders();
        //   CameraDevice.getInstance().setFlashTorchMode(true);

        int setA = 1;
        int setB = 1;

        if (strafeLeft) {
            setA = -1;
            setB = 1;
        } else {
            setA = 1;
            setB = -1;
        }

        // Determine new target position, and pass to motor controller;
        driveLF.setTargetPosition(encoderValue * setA);
        driveRF.setTargetPosition(encoderValue * setB);
        driveLR.setTargetPosition(encoderValue * setB);
        driveRR.setTargetPosition(encoderValue * setA);


        // Turn On RUN_TO_POSITION
        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int startDecelerationAt = (int) (encoderValue * .95);


        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (driveLF.isBusy() && driveRF.isBusy()
                        && driveLR.isBusy() && driveRR.isBusy())
        ) {

            currentPower = power;//rampUpDown(power,currentPower,.1,driveRR.getCurrentPosition(),startDecelerationAt);

            if(pid){
                driveRR.setPower(currentPower - correction);
                driveLR.setPower(currentPower + correction);
                driveLF.setPower(currentPower - correction);
                driveRF.setPower(currentPower + correction);
            } else {
                driveRR.setPower(currentPower);
                driveLR.setPower(currentPower);
                driveLF.setPower(currentPower);
                driveRF.setPower(currentPower);
            }

/*
            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();
                if (blockLoc != null) {
                    break;
                }
            }
 */
        }

        // Stop all motion;
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLR.setPower(0);
        driveRR.setPower(0);


        // Turn off RUN_TO_POSITION
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //     CameraDevice.getInstance().setFlashTorchMode(false);

        return blockLoc;
    }


    public double rampUpDown(double maxPower, double curPower, double minPower, int curPosition, int decelStartPosition) {

        double returnPower;

        if (Math.abs(curPosition) < Math.abs(decelStartPosition) && curPower < maxPower) {
            returnPower = curPower + .01;
        } else if (Math.abs(curPosition) >= Math.abs(decelStartPosition) && curPower > minPower) {
            returnPower = curPower - .01;
        } else {
            returnPower = curPower;
        }

        return returnPower;
    }


    public VectorF strafeToDistance(double slideSlowPower, double timeoutS, double distance,
                                    DistanceSensor sensor, boolean objectDetection) {

        DrivetrainCommon_ALT1.rotation = getAngle();        // reset angle tracking on new heading.
        DrivetrainCommon_ALT1.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction = 0;

        runtime.reset();

        VectorF blockLoc = null;

        if (objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(true);
        }

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {


            if ((slideSlowPower > 0 && sensor.getDistance(DistanceUnit.CM) > distance) ||
                    (slideSlowPower < 0 && sensor.getDistance(DistanceUnit.CM) < distance)) {
                break;
            }


            correction = DrivetrainCommon_ALT1.pidDrive.performPID(getAngle());
            //Front Motors
            powerLeftFront = -slideSlowPower - correction;
            powerRightFront = slideSlowPower + correction;

            //Rear Motors
            powerRightRear = -slideSlowPower + correction;
            powerLeftRear = slideSlowPower - correction;

            driveRR.setPower(powerRightRear);
            driveLR.setPower(powerLeftRear);
            driveLF.setPower(powerLeftFront);
            driveRF.setPower(powerRightFront);
/*
           if(objectDetection) {
               blockLoc = vuforiaCom.executeDetection();
               if (blockLoc != null) {
                   break;
               }
           }
 */
        }

        if (objectDetection) {
            CameraDevice.getInstance().setFlashTorchMode(false);
        }

        driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop all motion;
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLR.setPower(0);
        driveRR.setPower(0);


        // Turn off RUN_TO_POSITION
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        return blockLoc;
    }

    public VectorF strafeAwayDistance(double slideSlowPower, double timeoutS, double distance,
                                      DistanceSensor sensor, boolean objectDetection) {

        DrivetrainCommon_ALT1.rotation = getAngle();        // reset angle tracking on new heading.
        DrivetrainCommon_ALT1.resetAngle();

        double powerRightRear;
        double powerLeftRear;
        double powerLeftFront;
        double powerRightFront;

        double correction = 0;

        runtime.reset();

        VectorF blockLoc = null;

        CameraDevice.getInstance().setFlashTorchMode(true);

        while (curOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (sensor.getDistance(DistanceUnit.CM) < distance)) {

            correction = 0;//DrivetrainCommon_ALT1.pidDrive.performPID(DrivetrainCommon_ALT1.getAngle());
            //Front Motors
            powerLeftFront = -slideSlowPower - correction;
            powerRightFront = slideSlowPower + correction;

            //Rear Motors
            powerRightRear = -slideSlowPower + correction;
            powerLeftRear = slideSlowPower - correction;

            driveRR.setPower(powerRightRear);
            driveLR.setPower(powerLeftRear);
            driveLF.setPower(powerLeftFront);
            driveRF.setPower(powerRightFront);
/*
            if(objectDetection) {
                blockLoc = vuforiaCom.executeDetection();
                if (blockLoc != null) {
                    break;
                }
            }
 */
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

        driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop all motion;
        driveLF.setPower(0);
        driveRF.setPower(0);
        driveLR.setPower(0);
        driveRR.setPower(0);


        // Turn off RUN_TO_POSITIONssss
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        return blockLoc;
    }


    public int getDistFromHub(int pos, boolean red){
        if(red){
            if(pos == 3){
                return 1240;
            } else if (pos == 2){
                return 1160;
            } else if (pos == 1){
                return 980;
            } else{
                return 1240;
            }
        } else {
            if(pos == 3){
                return 1270;
            } else if (pos == 2){
                return 1190;
            } else if (pos == 1){
                return 1100;
            } else{
                return 1240;
            }
        }

    }

    boolean outerPassed = false;

    boolean isDone = false;

    public boolean scanForPole(boolean goingRight){
        double right = leftConeCheck.getDistance(DistanceUnit.INCH);
        double left = rightConeCheck.getDistance(DistanceUnit.INCH);
        double center = centerCheck.getDistance(DistanceUnit.INCH);

        if (goingRight) {
            if (right > 11.5 && !outerPassed) {
                executeDrive(.1,0);
            }
            if (right < 11.5) {
                outerPassed = true;
            }
            if (right > 11.5 && outerPassed) {
                executeDrive(0,0);
                isDone = true;
//                outerPassed = false;
            }
        } else {
            if (left > 11.5 && !outerPassed) {
                executeDrive(-.1,0);

            }
            if (left < 11.5) {
                outerPassed = true;
            }
            if (left > 11.5 && outerPassed) {
                executeDrive(0,0);
                isDone = true;
//                outerPassed = false;
            }
        }

        curOpMode.telemetry.addData("Left Check", left);
        curOpMode.telemetry.addData("Right Check", right);

        curOpMode.telemetry.update();

        return isDone;
    }

    public boolean isIn = false;
    public boolean junctionPassed = false;

    public boolean driveIntoPole(){
        if (junctionDriveDirCheck.getDistance(DistanceUnit.INCH) < 5) {
            junctionPassed = true;
        }
        if(!junctionPassed){
            executeDrive(0, .2);
        } else {
            executeDrive(0, 0);
            isIn = true;
        }

        return isIn;
    }

    public void driveToEnd(double driveSpeed, int pos, boolean high, boolean right){
        if(right) {
            if (high) {
                encoderStrafe(driveSpeed, 10, 500 + (pos * 1300), false, false, false);
            } else {
                if(pos == 0){
                    encoderStrafe(driveSpeed, 10, 500, true, false, false);
                } else {
                    encoderStrafe(driveSpeed, 10, 500 + (pos-1 * 1300), true, false, false);
                }
            }
        } else {
            if (high) {
                encoderStrafe(driveSpeed, 10, 500 + (Math.abs(pos-2) * 1300), true, false, false);
            } else {
                if(pos == 2){
                    encoderStrafe(driveSpeed, 10, 500, false, false, false);
                } else if (pos == 1){
                    encoderStrafe(driveSpeed, 10, 1800, true, false, false);
                } else if (pos == 0){
                    encoderStrafe(driveSpeed, 10, 3100, true, false, false);
                }
            }
        }
    }

}