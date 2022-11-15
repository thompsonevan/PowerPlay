
package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.Robot.DrivetrainLoopState;
import static org.firstinspires.ftc.teamcode.Robot.curOpMode;
import static org.firstinspires.ftc.teamcode.Robot.driver;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.centerCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveLF;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveLR;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveRF;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.driveRR;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.greenLed;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.greenLed2;

import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.imu;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.junctionDriveDirCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.leftConeCheck;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.redLed;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.redLed2;

import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware.rightConeCheck;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;
import org.firstinspires.ftc.teamcode.Robot;
import java.util.Locale;

public final class DrivetrainCommon_ALT1 {

    private double servoValue = 0;

    static Orientation angles;

    static int primaryHeading = 0;

    static Orientation lastAngles = new Orientation();
    public static double globalAngle;
    public static double currentAngle;
    public static double power = .30;
    public static double rotation;
    public static PIDController pidRotate;
    public static PIDController pidDrive;

    static double powerRightRear;
    static double powerLeftRear;
    static double powerLeftFront;
    static double powerRightFront;

    static double autoDrivePower = .2;
    static double autoStrafePower=.15;

    int autoPickLoopCount = 0;
    int stackPos=4;

    static double yVal = 0;
    static double xVal = 0;

    static double xVal_rs=0;

    static double turnVal = 0;
    double diagVal = 0;

    //static double strafeVal =0;
   // static double driveVal=0;

    static double min =0 ;
    static double setPointAngle = 0;

    static double slowPower = .25;
    double slideSlowPower = .25;
    static double slowTurn =.2;

    static double max = .7;
    static double turnMax = .5;

    // Important Stuff:
    double maxSpeed = 1;
    double controllerCap = .95;
    double speed = .75;

    //Variables for refining responsiveness of joystick controls
    static double powerShift = 0;
    static double expVal = 3;

    static boolean isPickupDropGood = false;
    static boolean isLiftUp = false;
    static boolean autoPickDropEnabled = false;
    static boolean startAutoPickDrop;
    static boolean atPickupLocation=false;
    static boolean autoRight = false;
    static boolean autoLeft = false;
    double baseLineDist;
    static double leftMax;
    static double rightMax;
    static double centerMin;
    double foreMin;
    double aftMin;

    static double leftVal;
    static double rightVal;
    static double centerVal;
    double foreVal;
    double aftVal;

    static boolean rampDrive;
    static boolean rampStrafe;
    static int rampInterval=25;
    static double rampIncrement=.01;
    static double rampTargetDriveSpeed = 0;
    static double rampTargetStrafeSpeed=0;
    static ElapsedTime rampRuntime= new ElapsedTime();

    static boolean autoSequenceRunning=false;

    static double correction = 0;
    static double leftTurnCorrection = 0;
    static double rightTurnCorrection = 0;

    public static void initDrivetrainCommon_ALT1() {

        DrivetrainHardware.initDrivetrainHardware();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //vuforia = new VuforiaCommon(curOpMode);

        pidRotate = new PIDController(0, 0, 0);
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(setPointAngle);
        pidDrive.setOutputRange(0, .9);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();

        resetAngle();

        composeTelemetry();
        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.update();
    }

    public static void executeDrivetrainTeleop() {

        DrivetrainLoopState = Robot.LoopStates.Running;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentAngle=angles.firstAngle;

        if(driver.back)
        {
            imu.initialize(DrivetrainHardware.parameters);
        }

        if(driver.start && driver.a)
        {
            driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(driver.x)
        {
            turnToAngle(-90);
            primaryHeading=-90;
        }
        if(driver.b) {
            turnToAngle(90);
            primaryHeading=90;
        }
        if(driver.a)
        {
            turnToAngle(0);
            primaryHeading=0;
        }
        if(driver.y)
        {
            turnToAngle(180);
            primaryHeading=180;
        }

        xVal = curOpMode.gamepad1.right_stick_x;
        yVal = curOpMode.gamepad1.left_stick_y;


        if (driver.left_trigger > 0) {
            xVal_rs = -driver.left_trigger;
        } else if (driver.right_trigger > 0) {
            xVal_rs = driver.right_trigger;
        } else {
            xVal_rs = 0;
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

                        driveRR.setPower(powerRightRear);
                        driveLR.setPower(powerLeftRear);
                        driveLF.setPower(powerLeftFront);
                        driveRF.setPower(powerRightFront);
                    } else if (Math.abs(curOpMode.gamepad1.right_stick_x) == 0) {

                        driveRR.setPower(0);
                        driveLR.setPower(0);
                        driveLF.setPower(0);
                        driveRF.setPower(0);
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
                if(!autoSequenceRunning) {
                    isPickupDropGood = checkCones();
                }
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

            if(!autoSequenceRunning) {
                executeDriveToFieldCoordinates(xVal, yVal);
            }

        }

        DrivetrainLoopState = Robot.LoopStates.Completed;
    }
    public static void getCorrection(double xVal, double yVal, int curHeading) {

        double powerVal = Math.max(Math.abs(xVal), Math.abs(yVal));

        double correctionValue = .05 * powerVal;

        int headingBoundary = 20;
        if ((angles.firstAngle > (curHeading - headingBoundary)
                && angles.firstAngle < (curHeading + headingBoundary))
            || (curHeading==180 && (angles.firstAngle>160 ||
                angles.firstAngle<-160)))
        {

            double curDiff = angles.firstAngle - curHeading;

            if (curHeading == 180 && angles.firstAngle < 0) {
                curDiff = curHeading + angles.firstAngle;
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


        public static void executeDriveToFieldCoordinates(double xVal, double yVal) {

            getCorrection(xVal,yVal,primaryHeading);

            double driveVal;
            double strafeVal;

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
            else
            {
                driveVal=yVal;
                strafeVal=xVal;
            }

            executeDrive(strafeVal,driveVal);
        }
    public static void executeDrive(double strafeVal, double driveVal) {
    //Drive forward
        //  double driveVal;
       //   double strafeVal;


       /**    if(Math.abs(inDriveVal)>.1 && (Math.abs(rampTargetDriveSpeed)<=Math.abs(inDriveVal))) {

               if(rampRuntime.milliseconds()>rampInterval) {
                   rampTargetDriveSpeed = Math.signum(inDriveVal)*(Math.abs(rampTargetDriveSpeed) + rampIncrement);
                   rampRuntime.reset();
               }


               driveVal=rampTargetDriveSpeed;

           }
           else if(Math.abs(rampTargetDriveSpeed)>=Math.abs(inDriveVal))
           {
               driveVal=rampTargetDriveSpeed;

           }
           else
           {
               rampTargetDriveSpeed=0.1;
               driveVal=inDriveVal;
           }

           if(Math.abs(inStrafeVal) >.1 && Math.abs(rampTargetStrafeSpeed)<=Math.abs(inStrafeVal))
           {
               if(rampRuntime.milliseconds()>rampInterval)
               {
                   rampTargetStrafeSpeed=Math.signum(inStrafeVal)*(Math.abs(rampTargetStrafeSpeed)+rampIncrement);
                   rampRuntime.reset();
               }
               strafeVal=rampTargetStrafeSpeed;
           }
           else if(Math.abs(rampTargetStrafeSpeed)>=Math.abs(inStrafeVal))
           {
               strafeVal=rampTargetStrafeSpeed;
           }
           else
           {
               rampTargetStrafeSpeed=0.1;
               strafeVal=inStrafeVal;
           }**/

            if (driveVal > 0) {

                //Left motors
                powerLeftRear = driveVal + leftTurnCorrection;
                powerLeftFront = driveVal + leftTurnCorrection;

                //Right Motors
                powerRightRear = driveVal - rightTurnCorrection;
                powerRightFront = driveVal - rightTurnCorrection;

            }
            //Drive backward
            else if (driveVal < 0) {
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

            driveRR.setPower(powerRightRear);
            driveLR.setPower(powerLeftRear);
            driveLF.setPower(powerLeftFront);
            driveRF.setPower(powerRightFront);

            //printData();
    }

    public static void autoPickDrop()
    {

        LiftClawCommon.reduceConeStackCount();

        LiftClawCommon.clearConeStack(false);

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
                //Clear,drive forward
                if(checkCones() && curOpMode.opModeIsActive() && curOpMode.gamepad2.right_stick_button
                        && centerVal > 1.3) {

                    getCorrection(0,autoDrivePower,0);
                    executeDrive(0, autoDrivePower);
                }
                //Shift Right
                if (!checkCones() && leftVal < rightVal) {

                         getCorrection(autoStrafePower,0,0);
                         executeDrive(autoStrafePower, 0);

                }
                //Shift Left
                if (!checkCones() && leftVal > rightVal) {
                    getCorrection(-autoStrafePower,0,0);
                    executeDrive(-autoStrafePower, 0);
                }
        }
        executeDrive(0,0);

       if(curOpMode.gamepad2.right_stick_button && autoPickDropEnabled && centerVal<1.3) {


           pickFromStack();

       }

    }

    public static void pickFromStack()
    {

        autoPickDropEnabled=false;
        autoRight=false;
        autoLeft=false;



        LiftClawCommon.nextConeInStack(false);

        LiftClawCommon.closeClaw();
      //  curOpMode.sleep(1000);
        LiftClawCommon.clearConeStack(false);

        while (curOpMode.gamepad2.right_stick_button)
        {
            getCorrection(0,-.3,0);
            executeDrive(0, -.3);

        }
    }
    public static boolean checkCones() {

        leftVal = leftConeCheck.getDistance(DistanceUnit.INCH);
        rightVal = rightConeCheck.getDistance(DistanceUnit.INCH);
        centerVal = centerCheck.getDistance(DistanceUnit.INCH);

        boolean clear = false;

        double distanceThreshold = 20;

        if((
                //(( (centerVal - rightVal) <-1.5)  && ((centerVal-leftVal) <-1.5)) ||
                (((centerVal+1.5)<rightVal)  && ((centerVal+1.5)<leftVal)) )
                && centerVal<distanceThreshold)
        {
            greenLed.setState(true);
            redLed.setState(false);

            greenLed2.setState(true);
            redLed2.setState(false);



            clear = true;
            autoPickDropEnabled=true;
        }

        else if ((Math.abs(leftVal-rightVal)>1.5)
            && (leftVal<distanceThreshold || rightVal<distanceThreshold))
        {
            //need to shift left
            if(leftVal<rightVal) {

                greenLed.setState(false);
                redLed.setState(true);


                greenLed2.setState(true);
                redLed2.setState(false);

            }
            //need to shift right
            else if(leftVal>rightVal)
            {
                greenLed.setState(true);
                redLed.setState(false);

                greenLed2.setState(false);
                redLed2.setState(true);

            }
            autoPickDropEnabled=true;
            clear=false;
        }
        else if((centerVal<distanceThreshold) && (centerVal<leftVal) && centerVal<rightVal)
        {
            greenLed.setState(true);
            redLed.setState(false);

            greenLed2.setState(true);
            redLed2.setState(false);

            clear = true;
            autoPickDropEnabled=true;
        }
        else
        {

            autoPickDropEnabled=false;
            greenLed.setState(false);
            redLed.setState(false);

            greenLed2.setState(false);
            redLed2.setState(false);
        }

        curOpMode.telemetry.addData("leftCheck:",leftVal);
        curOpMode.telemetry.addData("rightCheck:",rightVal);
        curOpMode.telemetry.addData("centerCheck:",centerVal);

        return clear;
    }

    public static void turnToAngle(double target)
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
            driveLF.setPower(-power*direction);
            driveLR.setPower(-power*direction);

            driveRF.setPower(power*direction);
            driveRR.setPower(power*direction);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if(target==180)
            {
                curDiff=Math.abs(angles.firstAngle)-target;
            }

            else {
                curDiff = angles.firstAngle - target;
            }



        }

        // turn the motors off.
        driveLF.setPower(0);
        driveLR.setPower(0);

        driveRF.setPower(0);
        driveRR.setPower(0);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public static void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

        setPointAngle=lastAngles.firstAngle;


    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public static double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
                driveLF.setPower(-power);
                driveLR.setPower(-power);

                driveRF.setPower(power);
                driveRR.setPower(power);
                //curOpMode.sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                driveLF.setPower(-power);
                driveLR.setPower(-power);

                driveRF.setPower(power);
                driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                driveLF.setPower(-power);
                driveLR.setPower(-power);

                driveRF.setPower(power);
                driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        driveLF.setPower(0);
        driveLR.setPower(0);

        driveRF.setPower(0);
        driveRR.setPower(0);




        // wait for rotation to stop.
        curOpMode.sleep(500);
        rotation = getAngle();        // reset angle tracking on new heading.
        resetAngle();
    }



    static void composeTelemetry() {

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



    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void printData(){
        //curOpMode.telemetry.addData("Pos (inches)", "{X, Y, Z, Heading} = %.1f, %.1f, %.1f, %.1f",
        //vuforia.getX(), vuforia.getY(), vuforia.getZ(), vuforia.getHeading());
        curOpMode.telemetry.addData("Left Front: ", driveLF.getCurrentPosition());
        curOpMode.telemetry.addData("Left Rear: ", driveLR.getCurrentPosition());
        curOpMode.telemetry.addData("Right Front: ", driveRF.getCurrentPosition());
        curOpMode.telemetry.addData("Right Rear: ", driveRR.getCurrentPosition());
    }

    public LinearOpMode getCurOpMode() {
        return curOpMode;
    }
}
