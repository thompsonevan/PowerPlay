
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;

import java.util.Locale;

public class DrivetrainCommon_ALT1 {

    public DrivetrainHardware robot = new DrivetrainHardware();

   // VuforiaCommon vuforia;

    // LiftClawHardware(); /* HT 15358 */

    // LiftClawCommon(); /* HT 15358 */

    private double servoValue = 0;

    private LinearOpMode curOpMode = null;

    Orientation angles;

    int primaryHeading = 0;

    Orientation lastAngles = new Orientation();
    public double globalAngle, currentAngle, power = .30, rotation;
    public PIDController pidRotate, pidDrive;

    double powerRightRear;
    double powerLeftRear;
    double powerLeftFront;
    double powerRightFront;

    double autoDrivePower = .2;
    double autoStrafePower=.15;

    int autoPickLoopCount = 0;
    int stackPos=4;

    double yVal = 0;
    double xVal = 0;

    double xVal_rs=0;

    double turnVal = 0;
    double diagVal = 0;

    double strafeVal =0;
    double driveVal=0;

    double min =0 ;
    double setPointAngle = 0;

    double slowPower = .25;
    double slideSlowPower = .25;
    double slowTurn =.2;

    double max = .7;
    double turnMax = .5;

    Gamepad driverControl;


    // Important Stuff:
    double maxSpeed = 1;
    double controllerCap = .95;
    double speed = .75;

    //Variables for refining responsiveness of joystick controls
    double powerShift = 0;
    double expVal = 3;

    boolean isPickupDropGood = false;
    boolean isLiftUp = false;
    boolean autoPickDropEnabled = false;
    boolean startAutoPickDrop;
    boolean atPickupLocation=false;
    boolean autoRight = false;
    boolean autoLeft = false;
    double baseLineDist,leftMax,rightMax,centerMin,foreMin,aftMin;

    double leftVal, rightVal,centerVal, foreVal, aftVal;
    public LiftClawCommon liftClaw;

    boolean autoSequenceRunning=false;

    double correction = 0;
    double leftTurnCorrection = 0;
    double rightTurnCorrection = 0;


    public DrivetrainCommon_ALT1(LinearOpMode owningOpMode) {
        curOpMode = owningOpMode;
        initHardware();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //vuforia = new VuforiaCommon(curOpMode);

        pidRotate = new PIDController(0, 0, 0);
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(setPointAngle);
        pidDrive.setOutputRange(0, .9);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();

        resetAngle();

        curOpMode.telemetry.addData("Mode", "calibrating...");
        curOpMode.telemetry.update();


        //    while (!curOpMode.isStopRequested() && !robot.imu.isGyroCalibrated()) {
        //        curOpMode.sleep(50);
        //       curOpMode.idle();
        //   }

        composeTelemetry();
        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());

        curOpMode.telemetry.update();
    }

    public void executeTeleop() {

        curOpMode.telemetry.addData("Correction:", correction);
        //vuforia.detect();
        if(curOpMode.gamepad2.back)
        {
            stackPos=4;
        }

        if(curOpMode.gamepad1.back)
        {
            robot.imu.initialize(robot.parameters);
        }

        if(curOpMode.gamepad1.start && curOpMode.gamepad1.a)
        {

            robot.driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        currentAngle=angles.firstAngle;

        if(curOpMode.gamepad1.x)
        {
            turnToAngle(-90);
            primaryHeading=-90;
        }
        if(curOpMode.gamepad1.b) {
            turnToAngle(90);
            primaryHeading=90;
        }
        if(curOpMode.gamepad1.a)
        {
            turnToAngle(0);
            primaryHeading=0;
        }
        if(curOpMode.gamepad1.y)
        {
            turnToAngle(180);
            primaryHeading=180;
        }

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        xVal = curOpMode.gamepad1.right_stick_x;

        yVal = curOpMode.gamepad1.left_stick_y;


        if (curOpMode.gamepad1.left_trigger > 0) {
            xVal_rs = -curOpMode.gamepad1.left_trigger;
        } else if (curOpMode.gamepad1.right_trigger > 0) {
            xVal_rs = curOpMode.gamepad1.right_trigger;
        } else {
            xVal_rs = 0;
        }


       // correction = 0;
       // leftTurnCorrection = 0;
       // rightTurnCorrection = 0;

        //if (Math.abs(curOpMode.gamepad1.left_stick_x) > .25) {
       //     correction = curOpMode.gamepad1.left_stick_x * .05;
        //} else if (Math.abs(curOpMode.gamepad1.right_stick_y) > .25) {
       //     correction = curOpMode.gamepad1.right_stick_y * .05;
      //  }


        boolean yDir = true;

        if (Math.abs(yVal) > Math.abs(xVal)) {

            //  yDir = true;
        }


    {

            //Maintain minimum power value whenever stick is above zero
            if (Math.abs(yVal) > 0 && Math.abs(yVal) < min) {

                yVal = Math.signum(yVal) * min;
            }

            if (Math.abs(xVal) > 0 && Math.abs(xVal) < min) {

                xVal = Math.signum(xVal) * min;
            }

            if (Math.abs(xVal_rs) > 0 && Math.abs(xVal_rs) < min) {

                xVal_rs = Math.signum(xVal_rs) * min;
            }

            //Scale power based on exponential curve

            if (Math.abs(yVal) > 0) {
                yVal = Math.signum(yVal) * (Math.pow(Math.abs(yVal), expVal) + powerShift);
            }

            if (Math.abs(xVal) > 0) {
                xVal = Math.signum(xVal) * (Math.pow(Math.abs(xVal), expVal) + powerShift);
            }

            if (Math.abs(xVal_rs) > 0) {
                xVal_rs = Math.signum(xVal_rs) * (Math.pow(Math.abs(xVal_rs), expVal) + powerShift);
            }


            if (Math.abs(yVal) > 1) {
                yVal = Math.signum(yVal) * 1;
            }

            if (Math.abs(xVal) > 1) {
                xVal = Math.signum(xVal) * 1;
            }

            if (Math.abs(xVal_rs) > 1) {
                xVal_rs = Math.signum(xVal_rs) * 1;
            }

            //Scale resulting curve values to defined maximum
            yVal = yVal * max;
            xVal = xVal * max;
            xVal_rs = xVal_rs * turnMax;

            curOpMode.telemetry.addData("X_Input:", curOpMode.gamepad1.right_stick_x);
            curOpMode.telemetry.addData("X_Output:", xVal);
            curOpMode.telemetry.addData("Y_Input:", curOpMode.gamepad1.left_stick_y);
            curOpMode.telemetry.addData("Y_Output:", yVal);

            if (curOpMode.gamepad1.left_bumper) {
                xVal_rs = -slowTurn;
            } else if (curOpMode.gamepad1.right_bumper) {
                xVal_rs = slowTurn;
            }

            if (Math.abs(xVal_rs) > 0) {

                //This case is for a pure turn with no forward/backward or slide L/R motion
                if (Math.abs(yVal) == 0 && Math.abs(xVal) == 0) {

                    if (Math.abs(xVal_rs) > 0) {

                        turnVal = xVal_rs;

                        powerRightRear = -turnVal;
                        powerLeftRear = turnVal;
                        powerLeftFront = turnVal;
                        powerRightFront = -turnVal;

                        robot.driveRR.setPower(powerRightRear);
                        robot.driveLR.setPower(powerLeftRear);
                        robot.driveLF.setPower(powerLeftFront);
                        robot.driveRF.setPower(powerRightFront);
                    } else if (Math.abs(curOpMode.gamepad1.right_stick_x) == 0) {

                        robot.driveRR.setPower(0);
                        robot.driveLR.setPower(0);
                        robot.driveLF.setPower(0);
                        robot.driveRF.setPower(0);
                    }

                }

                //Otherwise capture a turn value to cause simultaneous turning while moving forward/backward or sliding left/right
                else {

                    turnVal = xVal_rs;

                    if (turnVal > 0) {
                        rightTurnCorrection = turnVal;
                    } else {
                        leftTurnCorrection = turnVal;
                    }
                }

                rotation = getAngle();        // reset angle tracking on new heading.
                resetAngle();
            }

            if (correction > 0) {
                rightTurnCorrection = correction;
            } else {
                leftTurnCorrection = correction;
            }


            if (curOpMode.gamepad1.dpad_up) {
                yVal = -slowPower;
            } else if (curOpMode.gamepad1.dpad_down) {
                yVal = slowPower;
            } else if (curOpMode.gamepad1.dpad_left) {
                xVal = -slowPower;
            } else if (curOpMode.gamepad1.dpad_right) {
                xVal = slowPower;
            }

            if (isLiftUp) {

            } else {
                isPickupDropGood = checkCones();
            }

            if (curOpMode.gamepad2.right_stick_button && autoPickDropEnabled && !autoSequenceRunning ) {
                autoSequenceRunning=true;
                if (startAutoPickDrop) {
                    leftMax = leftVal;
                    rightMax = rightVal;
                    centerMin = centerVal;

                }
                autoPickDrop();
                autoSequenceRunning=false;
            }
            else {
                autoLeft = false;
                autoRight = false;
                startAutoPickDrop = true;
                atPickupLocation=false;
            }

            curOpMode.telemetry.addData("CurrentAngle:", angles.firstAngle);


            if(!autoSequenceRunning) {
                executeDriveToFieldCoordinates(xVal, yVal, true);
            }

        }
    }
    public void getCorrection(double xVal, double yVal) {

        double powerVal = Math.max(Math.abs(xVal), Math.abs(yVal));

        double correctionValue = .05 * powerVal;

        int headingBoundary = 20;
        if ((angles.firstAngle > (primaryHeading - headingBoundary)
                && angles.firstAngle < (primaryHeading + headingBoundary))
            || (primaryHeading==180 && (angles.firstAngle>160 ||
                angles.firstAngle<-160)))
        {

            double curDiff = angles.firstAngle - primaryHeading;

            if (primaryHeading == 180 && angles.firstAngle < 0) {
                curDiff = primaryHeading + angles.firstAngle;
            }

            if(Math.abs(curDiff)>.5) {
                correction = correctionValue * Math.abs(curDiff);

                if((yVal>0 && curDiff>0) || (yVal<0 && curDiff<0) )
                {
                    correction=-correction;
                }
                else if((xVal<0 && curDiff>0) || (xVal>0 && curDiff<0))
                {
                    correction=-correction;
                }

            }
            else
            {
                correction=0;
                leftTurnCorrection=0;
                rightTurnCorrection=0;
            }

        }
        else
        {
            correction=0;
            leftTurnCorrection=0;
            rightTurnCorrection=0;
        }
    }


        public void executeDriveToFieldCoordinates(double xVal, double yVal,
                                  boolean yDir) {

            getCorrection(xVal,yVal);


            if (angles.firstAngle >= -45 && angles.firstAngle <= 45) {

                primaryHeading=0;

                driveVal = yVal;
                strafeVal = -xVal;
                correction = -correction;

            } else if (angles.firstAngle > 45 && angles.firstAngle < 135) {
                driveVal = xVal;
                strafeVal = yVal;
                correction = -correction;
                rightTurnCorrection = -rightTurnCorrection;
                leftTurnCorrection = -leftTurnCorrection;
                primaryHeading=90;

            } else if (angles.firstAngle < -45 && angles.firstAngle > -135) {
                driveVal = -xVal;
                strafeVal = -yVal;
                rightTurnCorrection = -rightTurnCorrection;
                leftTurnCorrection = -leftTurnCorrection;

                primaryHeading=-90;

            } else if ((angles.firstAngle < -135 && angles.firstAngle >= -180)
                    || (angles.firstAngle <= 180 && angles.firstAngle > 135)) {
                driveVal = -yVal;
                strafeVal = xVal;

                primaryHeading=180;
            }

            executeDrive(strafeVal,driveVal,true);
        }
    public void executeDrive(double strafeVal, double driveVal,
    boolean yDir) {
    //Drive forward

            if (driveVal > 0 && yDir) {

                //Left motors
                powerLeftRear = driveVal + leftTurnCorrection;
                powerLeftFront = driveVal + leftTurnCorrection;

                //Right Motors
                powerRightRear = driveVal - rightTurnCorrection;
                powerRightFront = driveVal - rightTurnCorrection;

            }
            //Drive backward
            else if (driveVal < 0 && yDir) {
                //Left motors
                powerLeftRear = driveVal - rightTurnCorrection;
                powerLeftFront = driveVal - rightTurnCorrection;

                //Right Motors
                powerRightRear = driveVal + leftTurnCorrection;
                powerRightFront = driveVal + leftTurnCorrection;


            }

            //Slide Left/Right with no correction curve
            else if (Math.abs(strafeVal) > 0 && correction == 0) {

                //Front Motors
                powerLeftFront = strafeVal - correction;
                powerRightFront = -strafeVal + correction;

                //Rear Motors
                powerRightRear = strafeVal + correction;
                powerLeftRear = -strafeVal - correction;


            }
            //Slide Left/Right with correction causing curved motion
            else if (Math.abs(strafeVal) > 0 && Math.abs(correction) > 0) {
                //Slide Left with forward curve
                if (strafeVal < 0 && correction > 0) {
                    //Front Motors
                    powerLeftFront = strafeVal - correction;
                    powerRightFront = -strafeVal + correction;

                    //Rear Motors
                    powerRightRear = strafeVal;
                    powerLeftRear = -strafeVal;
                }
                //Slide Left with backward curve
                else if (strafeVal < 0 && correction < 0) {
                    //Front Motors
                    powerLeftFront = strafeVal;
                    powerRightFront = -strafeVal;

                    //Rear Motors
                    powerRightRear = strafeVal + correction;
                    powerLeftRear = -strafeVal - correction;
                }
                //Slide Right with backward curve
                else if (strafeVal > 0 && correction > 0) {
                    //Front Motors
                    powerLeftFront = strafeVal;
                    powerRightFront = -strafeVal;

                    //Rear Motors
                    powerRightRear = strafeVal - correction;
                    powerLeftRear = -strafeVal + correction;
                }
                //Slide right with forward curve
                else if (strafeVal > 0 && correction < 0) {
                    //Front Motors
                    powerLeftFront = strafeVal + correction;
                    powerRightFront = -strafeVal - correction;

                    //Rear Motors
                    powerRightRear = strafeVal;
                    powerLeftRear = -strafeVal;
                }
            } else if (Math.abs(xVal_rs) == 0) {
                powerRightRear = 0;
                powerLeftRear = 0;
                powerLeftFront = 0;
                powerRightFront = 0;
            }

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);

            printData();
    }

    private void initHardware() {

        robot.init(curOpMode.hardwareMap);
    }

    public void ReturnToZero(double speed) {

        pidRotate.setTolerance(.01);
        rotate(-angles.firstAngle,speed);

    }

    public void autoPickDrop()
    {

        autoPickLoopCount=0;

        liftClaw.clearConeStack(false);

        if(leftVal>leftMax)
        {
            leftMax=leftVal;
        }

        if(rightVal>rightMax)
        {
            rightMax=rightVal;
        }

        if(centerVal<centerMin)
        {
            centerMin=centerVal;
        }


        curOpMode.telemetry.addData("centerMin:",centerMin);
        while(centerVal > 1.3 && curOpMode.gamepad2.right_stick_button
                && curOpMode.opModeIsActive() && autoPickDropEnabled) {
                if(checkCones() && curOpMode.opModeIsActive() && curOpMode.gamepad2.right_stick_button
                        && centerVal > 1.3) {
                    executeDrive(0, autoDrivePower,  true);
                }
                if (!checkCones() && leftVal < rightVal) {
                         executeDrive(autoStrafePower, 0, true);

                }
                //Shift Left
                if (!checkCones() && leftVal > rightVal) {
                    executeDrive(-autoStrafePower, 0, true);
                }
        }
        executeDrive(0,0,true);

       if(curOpMode.gamepad2.right_stick_button) {

           autoPickLoopCount++;

                pickFromStack();

        }

    }

    public void pickFromStack()
    {

        autoPickDropEnabled=false;
        autoRight=false;
        autoLeft=false;

        liftClaw.nextConeInStack(false, liftClaw.pos);

        if(liftClaw.pos>1) {
            liftClaw.pos--;
        }
        else
        {
            liftClaw.pos=1;
        }
        autoPickLoopCount++;

        curOpMode.telemetry.addData("liftLoop",liftClaw.autoLiftLoopCount);
        curOpMode.telemetry.addData("pickLoop",autoPickLoopCount);
        curOpMode.telemetry.update();

        liftClaw.closeClaw();
      //  curOpMode.sleep(1000);
        liftClaw.clearConeStack(false);

        while (curOpMode.gamepad2.right_stick_button)
        {
            executeDrive(0, -.3,  true);

        }
    }
    public boolean checkCones() {

        leftVal = robot.leftConeCheck.getDistance(DistanceUnit.INCH);
        rightVal = robot.rightConeCheck.getDistance(DistanceUnit.INCH);
        centerVal = robot.centerCheck.getDistance(DistanceUnit.INCH);

        boolean clear = false;

        double distanceThreshold = 20;

        if((
                //(( (centerVal - rightVal) <-1.5)  && ((centerVal-leftVal) <-1.5)) ||
                (((centerVal+1.5)<rightVal)  && ((centerVal+1.5)<leftVal)) )
                && centerVal<distanceThreshold)
        {
            robot.greenLed.setState(true);
            robot.redLed.setState(false);

            robot.greenLed2.setState(true);
            robot.redLed2.setState(false);

            robot.greenLed3.setState(true);
            robot.redLed3.setState(false);


            robot.greenLed4.setState(true);
            robot.redLed4.setState(false);

            clear = true;
            autoPickDropEnabled=true;
        }

        else if ((Math.abs(leftVal-rightVal)>1.5)
            && (leftVal<distanceThreshold || rightVal<distanceThreshold))
        {
            //need to shift left
            if(leftVal<rightVal) {

                robot.greenLed.setState(false);
                robot.redLed.setState(true);

                robot.greenLed3.setState(false);
                robot.redLed3.setState(true);

                robot.greenLed2.setState(true);
                robot.redLed2.setState(false);

                robot.greenLed4.setState(true);
                robot.redLed4.setState(false);


            }
            //need to shift right
            else if(leftVal>rightVal)
            {
                robot.greenLed.setState(true);
                robot.redLed.setState(false);

                robot.greenLed3.setState(true);
                robot.redLed3.setState(false);

                robot.greenLed2.setState(false);
                robot.redLed2.setState(true);

                robot.greenLed4.setState(false);
                robot.redLed4.setState(true);

            }
            autoPickDropEnabled=true;
            clear=false;
        }
        else if((centerVal<distanceThreshold) && (centerVal<leftVal) && centerVal<rightVal)
        {
            robot.greenLed.setState(true);
            robot.redLed.setState(false);

            robot.greenLed2.setState(true);
            robot.redLed2.setState(false);

            robot.greenLed3.setState(true);
            robot.redLed3.setState(false);


            robot.greenLed4.setState(true);
            robot.redLed4.setState(false);

            clear = true;
            autoPickDropEnabled=true;
        }
        else
        {

            autoPickDropEnabled=false;
            robot.greenLed.setState(false);
            robot.redLed.setState(false);

            robot.greenLed2.setState(false);
            robot.redLed2.setState(false);

            robot.greenLed3.setState(false);
            robot.redLed3.setState(false);


            robot.greenLed4.setState(false);
            robot.redLed4.setState(false);
        }

        curOpMode.telemetry.addData("leftCheck:",leftVal);
        curOpMode.telemetry.addData("rightCheck:",rightVal);
        curOpMode.telemetry.addData("centerCheck:",centerVal);
        curOpMode.telemetry.addData("junctionCheck", robot.junctionDriveDirCheck.getDistance(DistanceUnit.INCH));

        return clear;
    }

    public void turnToAngle(double target)
    {

        double curDiff = angles.firstAngle-target;
        int direction;

        if(target==180 && (angles.firstAngle>-180 && angles.firstAngle<0))
        {
            direction=-1;
        }
        else if(target==-90 && (angles.firstAngle<-170 || angles.firstAngle>170))
        {
            direction=1;
        }
        else if(target==90 && (angles.firstAngle<-170 || angles.firstAngle>170))
        {
            direction=-1;
        }
        else if(curDiff>=0)
        {
            direction=-1;
        }
        else
        {
            direction=1;
        }

        power = .5;

        while(curOpMode.opModeIsActive() && Math.abs(curDiff)>1)
        {
            if(Math.abs(curDiff)<30)
            {
                power=.1;
            }
            robot.driveLF.setPower(-power*direction);
            robot.driveLR.setPower(-power*direction);

            robot.driveRF.setPower(power*direction);
            robot.driveRR.setPower(power*direction);

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if(target==180)
            {
                curDiff=Math.abs(angles.firstAngle)-target;
            }

            else {
                curDiff = angles.firstAngle - target;
            }



        }

        // turn the motors off.
        robot.driveLF.setPower(0);
        robot.driveLR.setPower(0);

        robot.driveRF.setPower(0);
        robot.driveRR.setPower(0);

    }



    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

        setPointAngle=lastAngles.firstAngle;


    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        pidRotate.reset();

        double p = Math.abs((float)power*((float)degrees/(float)180));
        double i = p / 100.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(-power, power);
        //pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
        pidRotate.setTolerance((1.0 / Math.abs(degrees)) * 100.0);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (curOpMode.opModeIsActive() && getAngle() == 0)
            {
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);
                //curOpMode.sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.driveLF.setPower(0);
        robot.driveLR.setPower(0);

        robot.driveRF.setPower(0);
        robot.driveRR.setPower(0);




        // wait for rotation to stop.
        curOpMode.sleep(500);
        rotation = getAngle();        // reset angle tracking on new heading.
        resetAngle();
    }



    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        curOpMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
               // angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
        });

        curOpMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void printData(){
        //curOpMode.telemetry.addData("Pos (inches)", "{X, Y, Z, Heading} = %.1f, %.1f, %.1f, %.1f",
        //vuforia.getX(), vuforia.getY(), vuforia.getZ(), vuforia.getHeading());
        curOpMode.telemetry.addData("Left Front: ", robot.driveLF.getCurrentPosition());
        curOpMode.telemetry.addData("Left Rear: ", robot.driveLR.getCurrentPosition());
        curOpMode.telemetry.addData("Right Front: ", robot.driveRF.getCurrentPosition());
        curOpMode.telemetry.addData("Right Rear: ", robot.driveRR.getCurrentPosition());
    }

    public LinearOpMode getCurOpMode() {
        return curOpMode;
    }
}
