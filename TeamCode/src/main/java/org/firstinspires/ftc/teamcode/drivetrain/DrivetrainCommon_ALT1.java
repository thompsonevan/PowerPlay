
package org.firstinspires.ftc.teamcode.drivetrain;

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

    Orientation lastAngles = new Orientation();
    public double globalAngle, currentAngle, power = .30, correction, rotation;
    public PIDController pidRotate, pidDrive;

    double powerRightRear;
    double powerLeftRear;
    double powerLeftFront;
    double powerRightFront;



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

    double max = .5;
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

    boolean autoRight = false;
    boolean autoLeft = false;
    double baseLineDist,leftMax,rightMax,centerMin,foreMin,aftMin;

    double leftVal, rightVal,centerVal, foreVal, aftVal;
    public LiftClawCommon liftClaw;

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
        //vuforia.detect();

        if(curOpMode.gamepad1.a)
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
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        xVal = curOpMode.gamepad1.right_stick_x;

        yVal = -curOpMode.gamepad1.left_stick_y;


        if (curOpMode.gamepad1.left_trigger > 0) {
            xVal_rs = -curOpMode.gamepad1.left_trigger;
        } else if (curOpMode.gamepad1.right_trigger > 0) {
            xVal_rs = curOpMode.gamepad1.right_trigger;
        } else {
            xVal_rs = 0;
        }


        correction = 0;
        double leftTurnCorrection = 0;
        double rightTurnCorrection = 0;

        if (Math.abs(curOpMode.gamepad1.left_stick_x) > 0) {
            correction = curOpMode.gamepad1.left_stick_x * .1;
        } else if (Math.abs(curOpMode.gamepad1.right_stick_y) > 0) {
            correction = curOpMode.gamepad1.right_stick_y * .1;
        }


        boolean yDir = true;

        if (Math.abs(yVal) > Math.abs(xVal)) {

            //  yDir = true;
        }


        if (curOpMode.gamepad1.x) {
            robot.driveLF.setPower(-.35);
            robot.driveRF.setPower(.35);
            robot.driveLR.setPower(.35);
            robot.driveRR.setPower(-.35);
        } else {

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
                yVal = slowPower;
            } else if (curOpMode.gamepad1.dpad_down) {
                yVal = -slowPower;
            } else if (curOpMode.gamepad1.dpad_left) {
                xVal = -slowPower;
            } else if (curOpMode.gamepad1.dpad_right) {
                xVal = slowPower;
            }

            if (isLiftUp) {

            } else {
                isPickupDropGood = checkCones();
            }

            if (curOpMode.gamepad2.right_stick_button && autoPickDropEnabled) {

                if (startAutoPickDrop) {
                    leftMax = leftVal;
                    rightMax = rightVal;
                    centerMin = centerVal;


                    startAutoPickDrop = false;
                }
                autoPickDrop();
            } else {
                autoLeft = false;
                autoRight = false;
                startAutoPickDrop = true;
            }


            curOpMode.telemetry.addData("CurrentAngle:", angles.firstAngle);


            executeDrive(xVal,yVal,leftTurnCorrection,rightTurnCorrection,correction,true);
        }
    }
        public void executeDrive(double xVal, double yVal,
                                  double leftTurnCorrection, double rightTurnCorrection, double correction,
                                  boolean yDir)
        {

            if (angles.firstAngle >= -45 && angles.firstAngle <= 45) {
                driveVal = -yVal;
                strafeVal = -xVal;
                correction = -correction;  //Strafe correction good, flip drive correction
                rightTurnCorrection = -rightTurnCorrection;
                leftTurnCorrection = -leftTurnCorrection;
            } else if (angles.firstAngle > 45 && angles.firstAngle < 135) {
                driveVal = xVal; //GOOD
                strafeVal = -yVal; //GOOD
                correction = -correction; //GOOD: Correction for Strafe & Drive Same
            } else if (angles.firstAngle < -45 && angles.firstAngle > -135) {
                driveVal = -xVal;
                strafeVal = yVal;

                //strafe correction good, flip drive correction
                rightTurnCorrection = -rightTurnCorrection;
                leftTurnCorrection = -leftTurnCorrection;
            } else if ((angles.firstAngle < -135 && angles.firstAngle >= -180)
                    || (angles.firstAngle <= 180 && angles.firstAngle > 135)) {
                driveVal = yVal;
                strafeVal = xVal;
            }

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

    public void ReturnToZero() {

        pidRotate.setTolerance(.01);
        rotate(-angles.firstAngle,.5);

    }


    public void turnToAngle(double speed, double targetAngle)
    {
        double curAngle = angles.firstAngle;

        double error = curAngle-targetAngle;


        while (curOpMode.opModeIsActive())
        {
            powerRightRear = -speed*Math.signum(error);
            powerLeftRear = speed*Math.signum(error);
            powerLeftFront = speed*Math.signum(error);
            powerRightFront = -speed*Math.signum(error);

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);

            error =  angles.firstAngle -targetAngle;

            if(Math.abs(error)<=2)
            {
                break;
            }

            curOpMode.telemetry.addData("error", error);
            curOpMode.telemetry.update();
        }

        robot.driveRR.setPower(0);
        robot.driveLR.setPower(0);
        robot.driveLF.setPower(0);
        robot.driveRF.setPower(0);

    }

    public void autoPickDrop()
    {

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

        double autoDrivePower = -.2;
        double autoStrafePower=.2;
        curOpMode.telemetry.addData("centerMin:",centerMin);
        if(centerVal > 1.3){
            if ((isPickupDropGood && !isLiftUp) || centerMin < 4 ){
                while(centerVal>1.3 && checkCones() && curOpMode.opModeIsActive() && curOpMode.gamepad2.right_stick_button) {
                    executeDrive(0, autoDrivePower, 0, 0, 0, true);
                }
            }
            else if (!isPickupDropGood && !isLiftUp) {
                //Shift Right
                if(leftVal<rightVal) {
                    while(!checkCones() && curOpMode.opModeIsActive() && curOpMode.gamepad2.right_stick_button)
                    {
                        executeDrive(-autoStrafePower,0,0,0,0,true);

                    }
                }
                //Shift Left
                if(leftVal>rightVal)
                {
                    while(!checkCones() && curOpMode.opModeIsActive() && curOpMode.gamepad2.right_stick_button)
                    {
                        executeDrive(autoStrafePower,0,0,0,0,true);

                    }
                }
            }
        }


        //Pickup cone
        else
        {
            autoPickDropEnabled=false;
            autoRight=false;
            autoLeft=false;

            liftClaw.closeClaw();

        }
    }
    public boolean checkCones() {

        leftVal = robot.leftConeCheck.getDistance(DistanceUnit.INCH);
        rightVal = robot.rightConeCheck.getDistance(DistanceUnit.INCH);
        centerVal = robot.centerCheck.getDistance(DistanceUnit.INCH);

        boolean clear = false;

        double distanceThreshold = 10;

        if((
                //(( (centerVal - rightVal) <-1.5)  && ((centerVal-leftVal) <-1.5)) ||
                (((centerVal+4)<rightVal)  && ((centerVal+4)<leftVal)) )
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

    /* HT 15358 some of this code belongs higher up, but these are all endgame related */

    public void endgamespin () /*code by HT 15358 */
    {


        while (curOpMode.gamepad1.a) { /* HT 15358 This is the endgamespin actual code, only active
                     while the gamepad button is depressed */
            double bumpVal; /* HT 15358 first drive into the build platform to be sure squared up */
            double revbumpAdj;  /* HT 15358 adj for backing away from platform */
            bumpVal = 0.2;
            powerRightRear = bumpVal;
            powerLeftRear = bumpVal;
            powerLeftFront = bumpVal;
            powerRightFront = bumpVal;

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);


            /* HT 15358 Fifth back away from the platform */
            revbumpAdj = 0.4;
            powerRightRear  = -revbumpAdj;
            powerLeftRear   = -revbumpAdj;
            powerLeftFront  = -revbumpAdj;
            powerRightFront = -revbumpAdj;

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);

                /* leftover HT 15358 that we might copy later
                            lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                            globalAngle = 0;

                            setPointAngle=lastAngles.firstAngle;
                */

        }

    } /*end of endgamespin */



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

}
