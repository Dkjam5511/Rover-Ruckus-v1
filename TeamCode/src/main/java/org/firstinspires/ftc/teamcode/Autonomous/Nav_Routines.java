package org.firstinspires.ftc.teamcode.Autonomous;


import android.provider.Settings;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalVariables;

import java.util.List;

import static org.firstinspires.ftc.teamcode.GlobalVariables.LIFT_DROP_TICKS;
import static org.firstinspires.ftc.teamcode.GlobalVariables.MAX_MIL1_TICKS;

abstract public class Nav_Routines extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor winchmotor;
    DcMotor mil1;
    DcMotor mil2;
    DcMotor liftmotor;
    Servo mineralslidesblockservo;
    Servo mineralboxservo;
    CRServo mineralintakeservo;
    Servo markerknockservo;
    Servo filterServo;
    DigitalChannel magneticlimitswitch;
    Rev2mDistanceSensor leftdistancesensor;
    Rev2mDistanceSensor backdistancesensor;
    Rev2mDistanceSensor rightdistancesensor;
    BNO055IMU imu;
    Orientation angles;

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    boolean gs_first_run = true;
    ElapsedTime gs_speed_timer = new ElapsedTime();

    private double wheel_encoder_ticks = 537.6;
    private double wheel_diameter = 3.75;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

    public double goforwardstopdetect = 2;

    int mil1startticks;
    int mil2startticks;
    int liftmotorstartticks;
    int winchstartticks;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    TFObjectDetector tfod;

    public void Nav_Init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        liftmotor = hardwareMap.dcMotor.get("lm");
        winchmotor = hardwareMap.dcMotor.get("wm");
        mil1 = hardwareMap.dcMotor.get("mil1");
        mil2 = hardwareMap.dcMotor.get("mil2");
        mineralslidesblockservo = hardwareMap.servo.get("msbs");
        mineralboxservo = hardwareMap.servo.get("mbs");
        mineralintakeservo = hardwareMap.crservo.get("mis");
        markerknockservo = hardwareMap.servo.get("mks");
        filterServo = hardwareMap.servo.get("fs");
        backdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "bds");
        leftdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "lds");
        rightdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "rds");
        magneticlimitswitch = hardwareMap.digitalChannel.get("lsw");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        winchmotor.setDirection(DcMotor.Direction.REVERSE);
        mil2.setDirection(DcMotor.Direction.REVERSE);
        liftmotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mil1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mil2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mil1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mil2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mil1startticks = mil1.getCurrentPosition();
        mil2startticks = mil2.getCurrentPosition();
        winchstartticks = winchmotor.getCurrentPosition();
        liftmotorstartticks = liftmotor.getCurrentPosition();

        leftdistancesensor.initialize();
        backdistancesensor.initialize();
        rightdistancesensor.initialize();

        mineralslidesblockservo.setPosition(GlobalVariables.SLIDE_BLOCK_OPEN_POS);
        markerknockservo.setPosition(1);
        filterServo.setPosition(GlobalVariables.FILTER_SERVO_NORMAL);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        parameters.vuforiaLicenseKey = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = .4;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
    }

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prevheading = 0;
        ElapsedTime timeouttimer = new ElapsedTime();

        DbgLog.msg("10435 Starting TURN_TO_HEADING");
        current_heading = currentheadingreading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeouttimer.reset();
        prevheading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeouttimer.seconds() < 2) {

            //wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;
            wheel_power = (1 * Math.pow((degrees_to_turn) / 15, 4) + 15) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }

            rightFront.setPower(wheel_power);
            rightRear.setPower(wheel_power);
            leftFront.setPower(-wheel_power);
            leftRear.setPower(-wheel_power);

            current_heading = currentheadingreading();

            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prevheading) > 1) {
                timeouttimer.reset();
                prevheading = current_heading;
            }

            DbgLog.msg("10435 TURN_TO_HEADING" + " go right: " + go_right + " degrees to turn: " + degrees_to_turn + " wheel power: " + wheel_power + " current heading: " + current_heading + "Wheel power: " + Double.toString(wheel_power));
        }

        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        DbgLog.msg("10435 ending TURN_TO_HEADING" + Double.toString(target_heading) + "  Final heading:" + Double.toString(current_heading) + "  After set power 0:" + Double.toString(angles.firstAngle));

    } // end of turn_to_heading

    public void go_forward(double inches_to_travel, double heading, double speed, boolean gotocrater) {

        DbgLog.msg("10435 Starting GO_FORWARD inches:" + Double.toString(inches_to_travel) + " heading:" + Double.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;
        boolean craterhit = false;

        ElapsedTime timeouttimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && timeouttimer.seconds() < goforwardstopdetect && !craterhit) {

            if (winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
                winchmotor.setPower(-1);
            } else {
                winchmotor.setPower(0);
            }
            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            if (gotocrater) {
                if (highest_ticks_traveled - lowest_ticks_traveled > 75) {
                    craterhit = true;
                }
            }

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.1) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 GO_FORWARD ticks_traveled: L:" + Double.toString(lowest_ticks_traveled_l)
                        + " R:" + Double.toString(lowest_ticks_traveled_r) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
                DbgLog.msg("10435 GO_FORWARD ticks_traveled: "
                        + " LF:" + Double.toString(ticks_traveled_l_Front)
                        + " RF:" + Double.toString(ticks_traveled_r_Front)
                        + " LR:" + Double.toString(ticks_traveled_l_Rear)
                        + " RR:" + Double.toString(ticks_traveled_r_Rear)
                );
            }

            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 GO_FORWARD slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        winchmotor.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending GO_FORWARD: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch))
                + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch))
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " timouttimer:" + Double.toString(timeouttimer.seconds())
                + " lowest ticks traveled:" + Integer.toString(lowest_ticks_traveled)
                + " highest ticks traveled:" + Integer.toString(highest_ticks_traveled));

    } // end of go_forward

    public void go_forward_retract(double inches_to_travel, double heading, double speed, boolean gotocrater, double direction_aggressiveness, double drop_marker_distance) {

        DbgLog.msg("10435 Starting GO_FORWARD_RETRACT inches:" + Double.toString(inches_to_travel) + " heading:" + Double.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;
        boolean craterhit = false;

        ElapsedTime timeouttimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && timeouttimer.seconds() < goforwardstopdetect && !craterhit) {

            if (liftmotor.getCurrentPosition() - liftmotorstartticks > GlobalVariables.LIFT_AUTO_RETRACT_TICKS) {
                liftmotor.setPower(-1);
            } else {
                liftmotor.setPower(0);
            }

            if (winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
                winchmotor.setPower(-1);
            } else {
                winchmotor.setPower(0);
            }

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading) * direction_aggressiveness;

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            if (lowest_ticks_traveled == 0) {
                lowest_ticks_traveled = (ticks_traveled_r_Front + ticks_traveled_r_Rear + ticks_traveled_l_Front + ticks_traveled_l_Rear) / 3; // just in case one encoder not working
            } else {
                lowest_ticks_traveled = (ticks_traveled_r_Front + ticks_traveled_r_Rear + ticks_traveled_l_Front + ticks_traveled_l_Rear) / 4;
            }

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            if (gotocrater) {
                if (highest_ticks_traveled - lowest_ticks_traveled > 75) {
                    craterhit = true;
                }
            }

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.1) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }


            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (drop_marker_distance > 0 && inches_to_travel - remaining_inches > drop_marker_distance) {
                markerknockservo.setPosition(0);
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 GO_FORWARD_RETRACT ticks_traveled:"
                        + " L:" + Double.toString(lowest_ticks_traveled_l)
                        + " R:" + Double.toString(lowest_ticks_traveled_r)
                        + " Distance Remaining:" + Double.toString(remaining_inches)
                        + " actual_speed:" + actual_speed
                        + " current speed:" + current_speed
                        + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
                DbgLog.msg("10435 GO_FORWARD_RETRACT ticks_traveled: "
                        + " LF:" + Double.toString(ticks_traveled_l_Front)
                        + " RF:" + Double.toString(ticks_traveled_r_Front)
                        + " LR:" + Double.toString(ticks_traveled_l_Rear)
                        + " RR:" + Double.toString(ticks_traveled_r_Rear)
                );
            }

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 GO_FORWARD_RETRACT slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        liftmotor.setPower(0);
        winchmotor.setPower(0);

        sleep(100);
        DbgLog.msg("10435 ending GO_FORWARD_RETRACT: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch))
                + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch))
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " timouttimer:" + Double.toString(timeouttimer.seconds())
                + " lowest ticks traveled:" + Integer.toString(lowest_ticks_traveled)
                + " highest ticks traveled:" + Integer.toString(highest_ticks_traveled));

    } // end of go_forward_retract

    public double go_sideways(double angledegrees, double heading, double power, double inches) {

        DbgLog.msg("10435 Starting go_sideways"
                + " angledegrees:" + Double.toString(angledegrees)
                + " heading:" + Double.toString(heading)
                + " power:" + Double.toString(power)
                + " inches:" + Double.toString(inches)
        );

        double stickpower = power;
        double angleradians;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        boolean mineralclose = false;
        boolean destinationreached = false;
        final double ticksperinch = 47;
        int ticks_to_travel;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticksperinch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (angledegrees < 270) {
            angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - angledegrees) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !mineralclose && !destinationreached) {

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            turningpower = -go_straight_adjustment(heading) * (power * 2);

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower;


            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);

            DbgLog.msg("10435 go_sideways"
                    + " turningpower:" + Double.toString(turningpower)
                    + " leftfrontpower" + Double.toString(leftfrontpower)
                    + " rightfrontpower" + Double.toString(rightfrontpower)
                    + " leftrearpower" + Double.toString(leftrearpower)
                    + " rightrearpower" + Double.toString(rightrearpower)
            );
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(50);

        return highest_ticks_traveled / ticksperinch;
    }

    public void gosidewaysretract(double angledegrees, double heading, double power, double inches) {
        DbgLog.msg("10435 Starting go_sideways retract"
                + " angledegrees:" + Double.toString(angledegrees)
                + " heading:" + Double.toString(heading)
                + " power:" + Double.toString(power)
                + " inches:" + Double.toString(inches)
        );
        int liftticks;
        double stickpower = power;
        double angleradians;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        boolean mineralclose = false;
        boolean destinationreached = false;
        final double ticksperinch = 47;
        int ticks_to_travel;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticksperinch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (angledegrees < 270) {
            angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - angledegrees) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !mineralclose && !destinationreached) {

            if (liftmotor.getCurrentPosition() - liftmotorstartticks > GlobalVariables.LIFT_AUTO_RETRACT_TICKS) {
                liftmotor.setPower(-1);
            } else {
                liftmotor.setPower(0);
            }

            if (winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
                winchmotor.setPower(-1);
            } else {
                winchmotor.setPower(0);
            }

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            turningpower = -go_straight_adjustment(heading) * (power * 2);

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower;


            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);

            DbgLog.msg("10435 go_sideways"
                    + " turningpower:" + Double.toString(turningpower)
                    + " leftfrontpower" + Double.toString(leftfrontpower)
                    + " rightfrontpower" + Double.toString(rightfrontpower)
                    + " leftrearpower" + Double.toString(leftrearpower)
                    + " rightrearpower" + Double.toString(rightrearpower)
            );
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        liftmotor.setPower(0);
        winchmotor.setPower(0);

        sleep(50);
    }

    public void go_sideways_to_wall(double heading, double power, double walldistance, boolean useleft) {
        ElapsedTime runtime = new ElapsedTime();
        double stickpower = power;
        double angleradians = 90;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower;
        double inchesreadfromwall;
        double angledegrees;

        if (useleft) {
            inchesreadfromwall = leftdistancesensor.getDistance(DistanceUnit.INCH);

        } else {
            inchesreadfromwall = rightdistancesensor.getDistance(DistanceUnit.INCH);
        }

        if (useleft && walldistance - inchesreadfromwall > 0 || !useleft && walldistance - inchesreadfromwall < 0) {
            angledegrees = 90;
        } else {
            angledegrees = 270;
        }

        DbgLog.msg("10435 Starting go_sideways_to_wall"
                + " angledegrees:" + Double.toString(angledegrees)
                + " heading:" + Double.toString(heading)
                + " power:" + Double.toString(power)
                + " walldistance:" + Double.toString(walldistance)
                + " useleft:" + Boolean.toString(useleft)
        );

        while (opModeIsActive() && Math.abs(inchesreadfromwall - walldistance) > 1 && runtime.seconds() < 2) {

            if (useleft) {
                inchesreadfromwall = leftdistancesensor.getDistance(DistanceUnit.INCH);
            } else {
                inchesreadfromwall = rightdistancesensor.getDistance(DistanceUnit.INCH);
            }

            if (useleft && walldistance - inchesreadfromwall > 0 || !useleft && walldistance - inchesreadfromwall < 0) {
                angledegrees = 90;
            } else {
                angledegrees = 270;
            }

            // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
            // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
            // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

            // This converts from *our* degrees to radians used by the mecanum power calcs.
            // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
            if (angledegrees == 90) {
                angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
            } else if (angledegrees == 270) {
                angleradians = (450 - angledegrees) * Math.PI / 180;
            }

            angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

            turningpower = -go_straight_adjustment(heading) * (power * 2);

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower;

            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);


            DbgLog.msg("10435 go_sideways_to_wall"
                    + " inchesreadfromwall: " + Double.toString(inchesreadfromwall)
                    + " turningpower:" + Double.toString(turningpower)
                    + " leftfrontpower" + Double.toString(leftfrontpower)
                    + " rightfrontpower" + Double.toString(rightfrontpower)
                    + " leftrearpower" + Double.toString(leftrearpower)
                    + " rightrearpower" + Double.toString(rightrearpower)
            );

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(50);
    }

    public double wallfollow(double inches_to_travel, double heading, double speed, double walldistance, boolean left, boolean finddepot) {

        DbgLog.msg("10435 Starting WALLFOLLOW inches:" + Double.toString(inches_to_travel) + " heading:" + Double.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;
        int average_ticks_traveled = 0;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;
        double distance_off = 0;
        boolean wallfound = false;

        ElapsedTime timeouttimer = new ElapsedTime();
        ElapsedTime distanceSensorTimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && timeouttimer.seconds() < goforwardstopdetect && !wallfound) {

            if (finddepot) {
                wallfound = checkbackdistancesensor();
            }

            if (left) {
                distance_off = leftdistancesensor.getDistance(DistanceUnit.INCH) - walldistance;
            } else {
                distance_off = rightdistancesensor.getDistance(DistanceUnit.INCH) - walldistance;
            }

            if (distanceSensorTimer.seconds() > 1) {
                if (Math.abs(distance_off) >= 1.5 && distance_off < 300) {
                    go_sideways_to_wall(heading, .4, walldistance, left);
                }
                distanceSensorTimer.reset();
            }
            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            average_ticks_traveled = (ticks_traveled_l_Front + ticks_traveled_l_Rear + ticks_traveled_r_Front + ticks_traveled_r_Rear) / 4;

            actual_speed = getSpeed(average_ticks_traveled);

            if (actual_speed > 0.1) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 WALLFOLLOW ticks_traveled: L:" + Double.toString(lowest_ticks_traveled_l)
                        + " R:" + Double.toString(lowest_ticks_traveled_r) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed + "Distance Off: " + distance_off);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
            }

            destination_reached = (average_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 WALLFOLLOW slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending WALLFOLLOW: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch))
                + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch))
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " timouttimer:" + Double.toString(timeouttimer.seconds())
                + " lowest ticks traveled:" + Integer.toString(lowest_ticks_traveled)
                + " highest ticks traveled:" + Integer.toString(highest_ticks_traveled));

        return (inches_to_travel - 1) - (average_ticks_traveled / ticks_per_inch);
    }

    public void winchdown() {

        while (opModeIsActive() && winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
            winchmotor.setPower(-1);
        }
        winchmotor.setPower(0);
    }

    public void winchup() {
        boolean magnetistouching = false;

        while (!magnetistouching && opModeIsActive()) {
            magnetistouching = !magneticlimitswitch.getState();
            if (!magnetistouching) {
                winchmotor.setPower(1);
            } else {
                winchmotor.setPower(0);
            }
        }
    }

    public void deploymarker2() {
        markerknockservo.setPosition(0);
        sleep(500);
    }

    public void deploymineralarm() {
        int mil1ticks;
        int liftticks;
        double mil1tickspersec = 0;
        int prevmil1ticks = 0;
        double mineralboxpos;
        double variablespeed = 0;
        double inversespeed = 0;

        boolean scoopcomplete = false;

        ElapsedTime mil1tickpersectimer = new ElapsedTime();
        boolean deploying = true;

        while (deploying && opModeIsActive()) {

            if (winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
                winchmotor.setPower(-1);
            } else {
                winchmotor.setPower(0);
            }

            mil1ticks = mil1startticks - mil1.getCurrentPosition();
            liftticks = liftmotor.getCurrentPosition() - liftmotorstartticks;

            if (mil1ticks > GlobalVariables.MIL1_SCOOPING_TICKS) {
                mineralboxpos = GlobalVariables.MINERAL_BOX_INTAKE_POS;
                scoopcomplete = true;
            } else {
                mineralboxpos = GlobalVariables.MINERAL_BOX_BEGINSCOOP_POS;
            }

            mineralboxservo.setPosition(mineralboxpos);

            if (mil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS) {
                mineralslidesblockservo.setPosition(GlobalVariables.SLIDE_BLOCK_CLOSED_POS);
            }

            if (mil1tickpersectimer.seconds() > .05) {
                mil1tickspersec = (mil1ticks - prevmil1ticks) / mil1tickpersectimer.seconds();
                mil1tickpersectimer.reset();
            } else {
                prevmil1ticks = mil1ticks;
            }

            if (liftticks > GlobalVariables.LIFT_LOWERING_RETRACT_TICKS) {
                liftmotor.setPower(-1);    // retract mineral arm
            } else {
                liftmotor.setPower(0);
            }

            if (mil1ticks < GlobalVariables.MIL1_FULL_SPEED_DEPLOY_TICKS) {
                mil1.setPower(-1);
                mil2.setPower(-1);
            } else if (mil1ticks < GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS) {
                variablespeed = 1 - ((mil1ticks / GlobalVariables.MAX_MIL1_TICKS - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE + (1 - GlobalVariables.MIL1_BRAKING_DEPLOY_PERCENTAGE)) / (1 - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE));
                if (variablespeed < 0) {
                    variablespeed = 0;
                }
                inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND - mil1tickspersec;
                if (inversespeed < GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100) {
                    inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100;
                }
                inversespeed = inversespeed / GlobalVariables.MIL1_MAX_TICKS_PER_SECOND;
                mil1.setPower(-variablespeed * inversespeed);
                mil2.setPower(-variablespeed * inversespeed);
            } else {  // braking
                if (mil1ticks < GlobalVariables.MIL1_SCOOPING_TICKS) {
                    if (mil1tickspersec > 300) {
                        mil1.setPower(0);
                        mil2.setPower(0);
                    } else {
                        mil1.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);  // keep it moving until scooping ticks achieved
                        mil2.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                    }
                } else {
                    mil1.setPower(0);
                    mil2.setPower(0);
                    if (scoopcomplete) {
                        deploying = false;
                    }
                }
            }
        }
        mil1.setPower(0);
        mil2.setPower(0);
        winchmotor.setPower(0);
        liftmotor.setPower(0);

    }

    public void deploymineralarmextended(double liftextensionticks){
        double mil1ticks;
        double liftticks;
        double mil1tickspersec = 0;
        double prevmil1ticks = 0;
        double mineralboxpos;
        double variablespeed = 0;
        double inversespeed = 0;
        double brakingadjustmentticks = (liftextensionticks - GlobalVariables.LIFT_LOWERING_RETRACT_TICKS_BASE) / (GlobalVariables.LIFT_DROP_TICKS - GlobalVariables.LIFT_LOWERING_RETRACT_TICKS_BASE) * .025 * GlobalVariables.LIFT_DROP_TICKS;
        boolean scoopcomplete = false;

        DbgLog.msg("10435 Starting DEPLOYMINERALARMEXTENDED"
                        + " full speed deploy ticks:" + Double.toString(GlobalVariables.MIL1_FULL_SPEED_DEPLOY_TICKS)
                        + " braking deploy ticks:" + Double.toString(GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS)
                        + " scooping ticks:" + Double.toString(GlobalVariables.MIL1_SCOOPING_TICKS)
                        + " max ticks per second:" + Double.toString(GlobalVariables.MIL1_MAX_TICKS_PER_SECOND)
        );

        ElapsedTime mil1tickpersectimer = new ElapsedTime();
        boolean deploying = true;

        mil1ticks = mil1startticks - mil1.getCurrentPosition();
        prevmil1ticks = mil1ticks;

        while (deploying && opModeIsActive()) {

            if (winchmotor.getCurrentPosition() - winchstartticks > GlobalVariables.WINCH_RETRACT_TICKS) {
                winchmotor.setPower(-1);
            } else {
                winchmotor.setPower(0);
            }

            mil1ticks = mil1startticks - mil1.getCurrentPosition();
            liftticks = liftmotor.getCurrentPosition() - liftmotorstartticks;

            if (mil1ticks > GlobalVariables.MIL1_SCOOPING_TICKS) {
                mineralboxpos = GlobalVariables.MINERAL_BOX_INTAKE_POS;
                scoopcomplete = true;
            } else {
                mineralboxpos = GlobalVariables.MINERAL_BOX_BEGINSCOOP_POS;
            }

            mineralboxservo.setPosition(mineralboxpos);

            if (mil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS) {
                mineralslidesblockservo.setPosition(GlobalVariables.SLIDE_BLOCK_CLOSED_POS);
            }

            if (mil1tickpersectimer.seconds() > .03) {
                mil1tickspersec = (mil1ticks - prevmil1ticks) / mil1tickpersectimer.seconds();
                mil1tickpersectimer.reset();
                prevmil1ticks = mil1ticks;
            }

            if (liftticks > liftextensionticks) {
                liftmotor.setPower(-1);    // retract mineral arm
            } else {
                liftmotor.setPower(0);
            }

            DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED "
                    + " mil1 ticks:" + Double.toString(mil1ticks)
                    + " mil1 ticks per second:" + Double.toString(mil1tickspersec)
                    + " lift ticks:" + Double.toString(liftticks)
                    + " braking adjustment ticks:" + Double.toString(brakingadjustmentticks)
            );


            if (mil1ticks < GlobalVariables.MIL1_FULL_SPEED_DEPLOY_TICKS + .3 * MAX_MIL1_TICKS) {
                mil1.setPower(-.7);
                mil2.setPower(-.7);
            } else if (mil1ticks < GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_TICKS) {
                DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED medium speed power");
                mil1.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
                mil2.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
            } else if (mil1ticks < GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS - brakingadjustmentticks) {
                /* if (mil1tickspersec > GlobalVariables.MIL1_MAX_TICKS_PER_SECOND) {
                    DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED too fast - braking");
                    mil1.setPower(0);  // braking
                    mil2.setPower(0);
                } else */ {
                    DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED low speed power");
                    mil1.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                    mil2.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                }
             /*
             } else if (mil1ticks < GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS) {

                variablespeed = (1 - (((double)mil1ticks / GlobalVariables.MAX_MIL1_TICKS - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE + (1 - GlobalVariables.MIL1_BRAKING_DEPLOY_PERCENTAGE)) / (1 - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE)));
                if (variablespeed < 0) {
                    variablespeed = 0;
                }
                inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND - mil1tickspersec;
                if (inversespeed < GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100) {
                    inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100;
                }
                inversespeed = inversespeed / GlobalVariables.MIL1_MAX_TICKS_PER_SECOND;
                mil1.setPower(-variablespeed * inversespeed);
                mil2.setPower(-variablespeed * inversespeed);
                DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED "
                        + " mil1ticks:" + Integer.toString(mil1ticks)
                        + " inverse speed:" + Double.toString(inversespeed)
                        + " variable speed:" + Double.toString(variablespeed)
                        + " speed set:" + Double.toString(-variablespeed * inversespeed)
                );
            */
            } else {  // braking
                if (mil1ticks < GlobalVariables.MIL1_SCOOPING_TICKS) {
                    if (mil1tickspersec > 600) {
                        DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED Haven't reached scooping ticks - braking");
                        mil1.setPower(0);
                        mil2.setPower(0);
                    } else {
                        DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED Haven't reached scooping ticks - Setting power to -.07");
                        mil1.setPower(-.2);  // keep it moving until scooping ticks achieved
                        mil2.setPower(-.2);
                    }
                } else {
                    DbgLog.msg("10435 DEPLOYMINERALARMEXTENDED Scoop complete");
                    mil1.setPower(0);
                    mil2.setPower(0);
                    if (scoopcomplete) {
                        deploying = false;
                    }
                }
            }
        }

        winchmotor.setPower(0);
        liftmotor.setPower(0);
        mil1.setPower(0);
        mil2.setPower(0);

    }

    public void raisemineralarm() {
        int mil1ticks;
        int liftticks;
        int phase;
        boolean intakereverse = false;
        double boxmil1ticks;
        double mineralboxpos;
        final double mineralboxservoturnpos = .2; // the pos where the servo starts turning

        boolean raising = true;

        ElapsedTime boxtimer = new ElapsedTime();
        boxtimer.reset();

        while (raising && opModeIsActive()) {
            mil1ticks = mil1startticks - mil1.getCurrentPosition();
            liftticks = liftmotor.getCurrentPosition() - liftmotorstartticks;

            boxmil1ticks = mil1ticks;
            if (boxmil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS + GlobalVariables.MIL1_TICKS_TO_TURN_BOX) {   // set the max for boxmilticks
                boxmil1ticks = GlobalVariables.MIL1_DROP_LEVEL_TICKS + GlobalVariables.MIL1_TICKS_TO_TURN_BOX;
            }
            if (boxmil1ticks < GlobalVariables.MIL1_DROP_LEVEL_TICKS) {                     // set the min for boxmilticks
                boxmil1ticks = GlobalVariables.MIL1_DROP_LEVEL_TICKS;
            }
            mineralboxpos = ((boxmil1ticks - GlobalVariables.MIL1_DROP_LEVEL_TICKS) / GlobalVariables.MIL1_TICKS_TO_TURN_BOX) * (GlobalVariables.MINERAL_BOX_BEGINLIFT_POS - GlobalVariables.MINERAL_BOX_LIFTED_POS) + GlobalVariables.MINERAL_BOX_LIFTED_POS;

            mineralboxservo.setPosition(mineralboxpos);

            if (intakereverse) {
                mineralintakeservo.setPower(GlobalVariables.MINERAL_INTAKE_REVERSE_SPEED);
            } else {
                mineralintakeservo.setPower(GlobalVariables.MINERAL_INTAKE_FORWARD_SPEED);
            }

            intakereverse = false;

            if (mil1ticks > GlobalVariables.MIL1_RAISING_FULL_SPEED_TICKS) {
                phase = 1;
            } else if (mil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS) {
                phase = 2;
            } else {
                phase = 3;
            }

            if (phase == 1) {
                if (boxtimer.seconds() <= .4) {  // don't start phase 1 until box has a chance to turn and we sweep backwards
                    //intakereverse = true;
                } else {
                    if (boxtimer.seconds() <= .6) {  // don't start phase 1 until box has a chance to turn and we sweep backwards
                        intakereverse = true;
                    }
                    if (liftticks > GlobalVariables.LIFT_RAISING_RETRACT_TICKS) {
                        liftmotor.setPower(-1);
                    } else {
                        liftmotor.setPower(0);
                    }
                    mil1.setPower(1);
                    mil2.setPower(1);
                }
            }
            if (phase == 2) {
                if (liftticks < GlobalVariables.LIFT_DROP_TICKS) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                if (mil1ticks > GlobalVariables.MIL1_RAISING_SLOWER_TICKS) {
                    mil1.setPower(GlobalVariables.MIL1_RAISING_SLOWER_POWER);
                    mil2.setPower(GlobalVariables.MIL1_RAISING_SLOWER_POWER);
                } else if (mil1ticks > GlobalVariables.MIL1_RAISING_SLOWEST_TICKS) {
                    mil1.setPower(GlobalVariables.MIL1_RAISING_SLOWEST_POWER);
                    mil2.setPower(GlobalVariables.MIL1_RAISING_SLOWEST_POWER);
                } else {
                    mil1.setPower(GlobalVariables.MIL1_HOLD_DROP_POSITION_POWER);
                    mil2.setPower(GlobalVariables.MIL1_HOLD_DROP_POSITION_POWER);
                }
            }

            if (phase == 3) {
                mil1.setPower(0);
                mil2.setPower(0);
                if (liftticks < GlobalVariables.LIFT_DROP_TICKS) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                    raising = false;
                }
            }
        }
    }

    public void extendmineralarm(double extend_ticks) {
        int liftticks;
        boolean extending = true;

        while (extending && opModeIsActive()) {
            liftticks = liftmotor.getCurrentPosition() - liftmotorstartticks;
            if (liftticks < extend_ticks) {
                liftmotor.setPower(.5);
            } else {
                liftmotor.setPower(0);
                extending = false;
            }
        }
        liftmotor.setPower(0);
    }

    public void dumpmineral(boolean halfdump) {
        if (halfdump){
            mineralboxservo.setPosition(GlobalVariables.MINERAL_BOX_HALF_DROP);
            sleep((long)(1000 * (GlobalVariables.MINERAL_BOX_HALFDROP_TURN_TIME + GlobalVariables.MINERAL_BOX_HALFDROP_PAUSE_TIME)));
        }
        mineralboxservo.setPosition(GlobalVariables.MINERAL_BOX_FULL_DROP);
        sleep(1000);
    }

    public boolean checktfod() {
        boolean found;
        int loopcount = 0;

        telemetry.update();  // clear out telemetry?
        int closestmineral = 0;
        double closestbottom = 0;
        String closestlabel = "";

        while (opModeIsActive() && loopcount < 5) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        int mineralcounter;
                        mineralcounter = 0;
                        closestmineral = 0;
                        closestbottom = 0;
                        loopcount = loopcount + 1;
                        for (Recognition recognition : updatedRecognitions) {
                            mineralcounter = mineralcounter + 1;
                            telemetry.addData("Mineral counter", mineralcounter);
                            telemetry.addData("Label", recognition.getLabel());
                            telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            telemetry.addData("Width", recognition.getWidth());
                            telemetry.addData("Bottom", recognition.getBottom());
                            if (loopcount > 2 && recognition.getBottom() > closestbottom) {
                                closestbottom = recognition.getBottom();
                                closestmineral = mineralcounter;
                                closestlabel = recognition.getLabel();
                            }

                        }

                    }
                }
            }

            telemetry.addData("loopcount", loopcount);
            telemetry.addData("Closest Mineral", closestmineral);
            telemetry.addData("Closest Bottom", closestbottom);
            telemetry.addData("Closest Label", closestlabel);
            telemetry.update();
            sleep(300);
        }

        found = (closestbottom > 0 && closestlabel == "Gold Mineral");
        telemetry.addData("Found", found);

        telemetry.update();

        return found;
    }

    public int checktfod2() {
        int LCR = 1;  // 1=left 2=center 3=right
        int loopcount = 0;

        telemetry.update();  // clear out telemetry?
        int closestmineral1 = 0;
        int closestmineral2 = 0;
        double closestbottom1 = 0;
        double closestbottom2 = 0;
        String closestlabel1 = "";
        String closestlabel2 = "";
        double closestLeft1 = 0;
        double closestLeft2 = 0;

        ElapsedTime loopruntime = new ElapsedTime();
        while (opModeIsActive() && loopcount < 5 && loopruntime.seconds() < 3) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() >= 2) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        int mineralcounter;
                        mineralcounter = 0;
                        closestmineral1 = 0;
                        closestmineral2 = 0;
                        closestbottom1 = 0;
                        closestbottom2 = 0;
                        closestLeft1 = 0;
                        closestLeft2 = 0;
                        loopcount = loopcount + 1;
                        for (Recognition recognition : updatedRecognitions) {
                            mineralcounter = mineralcounter + 1;
                            DbgLog.msg(mineralcounter + " Bottom: " + recognition.getBottom()
                                    + " Label: " + recognition.getLabel()
                                    + " Left: " + recognition.getLeft()
                                    + " Width: " + recognition.getWidth()
                                    + " Height: " + recognition.getHeight()
                                    + " Angle: " + recognition.estimateAngleToObject(AngleUnit.DEGREES)
                            );
                            if (loopcount > 2) {
                                if (recognition.getBottom() > closestbottom1) { // We actually want the closest to the bottom. But bottom gets us distance from the top so bigger is closer.
                                    closestbottom2 = closestbottom1; // Move the smazller one to 2 so we keep 1 as the largest number (closest)
                                    closestmineral2 = closestmineral1;
                                    closestlabel2 = closestlabel1;
                                    closestLeft2 = closestLeft1;
                                    closestbottom1 = recognition.getBottom();
                                    closestmineral1 = mineralcounter;
                                    closestlabel1 = recognition.getLabel();
                                    closestLeft1 = recognition.getLeft();
                                } else if (recognition.getBottom() > closestbottom2) {
                                    closestbottom2 = recognition.getBottom();
                                    closestmineral2 = mineralcounter;
                                    closestlabel2 = recognition.getLabel();
                                    closestLeft2 = recognition.getLeft();
                                }
                            }
                        }

                    }
                }
            }

            telemetry.addData("loopcount", loopcount);
            telemetry.addData("Closest Mineral 1", closestmineral1);
            telemetry.addData("Closest Mineral 2", closestmineral2);
            telemetry.addData("Closest Bottom 1", closestbottom1);
            telemetry.addData("Closest Bottom 2", closestbottom2);
            telemetry.addData("Closest Label 1", closestlabel1);
            telemetry.addData("Closest Label 2", closestlabel2);
            telemetry.addData("Closest Left 1", closestLeft1);
            telemetry.addData("Closest Left 2", closestLeft2);
            telemetry.update();
            sleep(100); //This is to slow down the reads
        }

        if (Math.abs(closestbottom1 - closestbottom2) > 70) { //Throw out minerals that are too far back
            if (closestbottom1 < closestbottom2) {
                closestbottom1 = 0;
                DbgLog.msg("10435 Throwing out Mineral 1");
            } else {
                closestbottom2 = 0;
                DbgLog.msg("10435 Throwing out Mineral 2");
            }
        }

        if (closestbottom1 != 0 && closestlabel1 == "Gold Mineral") {
            if (closestLeft1 > 250) {
                LCR = 3;
            } else {
                LCR = 2;
            }
        } else if (closestbottom2 != 0 && closestlabel2 == "Gold Mineral") {
            if (closestLeft2 > 250) {
                LCR = 3;
            } else {
                LCR = 2;
            }
        } else {
            LCR = 1;
        }

        /*
        if (closestbottom1 != 0 && closestbottom2 != 0) {
            if (closestlabel1 == "Silver Mineral" && closestlabel2 == "Silver Mineral") {
                LCR = 1;
            } else {
                if (closestLeft1 < closestLeft2 && closestlabel1 == "Gold Mineral" || closestLeft2 < closestLeft1 && closestlabel2 == "Gold Mineral") { //If the closest left mineral is gold then set LCR to left
                    LCR = 2;
                } else {
                    LCR = 3;
                }
            }
        }
*/

        DbgLog.msg("Found: " + LCR
                + " loopcount: " + loopcount
                + " Closest Mineral 1: " + closestmineral1
                + " Closest Mineral 2: " + closestmineral2
                + " Closest Bottom 1: " + closestbottom1
                + " Closest Bottom 2: " + closestbottom2
                + " Closest Label 1: " + closestlabel1
                + " Closest Label 2: " + closestlabel2
                + " Closest Left 1: " + closestLeft1
                + " Closest Left 2: " + closestLeft2
        );
        telemetry.addData("Found: ", LCR);

        telemetry.update();

        return LCR;

    }

    public void deactivateTfod() {
        tfod.deactivate();
        vuforia = null;
    }

    private double getSpeed(double ticks_traveled) {
        double new_speed;

        if (gs_first_run) {
            gs_previous_ticks_traveled = ticks_traveled;
            gs_speed_timer.reset();
            gs_previous_speed = 1;
            gs_first_run = false;
        }

        if (gs_speed_timer.seconds() >= .1) {
            new_speed = (ticks_traveled - gs_previous_ticks_traveled) / 46.5;  // At max speed we travel about 4800 ticks in a second so this give a range of 0 - 10 for speed
            gs_speed_timer.reset();
            gs_previous_speed = new_speed;
            gs_previous_ticks_traveled = ticks_traveled;
            DbgLog.msg("10435 getspeed:" + Double.toString(new_speed));
        } else {
            new_speed = gs_previous_speed;
        }

        return new_speed;
    }

    private boolean checkbackdistancesensor() {
        boolean wallfound = false;

        if (backdistancesensor.getDistance(DistanceUnit.INCH) <= 22) {
            wallfound = true;
        }

        return wallfound;
    }

    private double go_straight_adjustment(double target_heading) {

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel

        double gs_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        current_heading = currentheadingreading();

        DbgLog.msg("10435 Starting go_straight_adjustment heading:" + Double.toString(target_heading) + " current heading:" + current_heading);

        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < .3) {
            gs_adjustment = 0;
        } else {
            gs_adjustment = (Math.pow((degrees_off + 2) / 5, 2) + 2) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        DbgLog.msg("10435 ending go_straight_adjustment adjustment:" + Double.toString(gs_adjustment));

        return gs_adjustment;

    }

    private double currentheadingreading() {
        double current_heading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = angles.firstAngle;

        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }

        current_heading = shiftheading(current_heading);

        return current_heading;
    }

    private double shiftheading(double heading) {
        double shiftvalue = 2;
        heading = heading + shiftvalue;

        if (heading >= 360) {
            heading = heading - 360;
        } else if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }
}

