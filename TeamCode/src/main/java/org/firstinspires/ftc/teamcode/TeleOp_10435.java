package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp_10435", group = "TeleOp")
public class TeleOp_10435 extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor winchMotor;
    DcMotor mil1;
    DcMotor mil2;
    DcMotor liftMotor;
    CRServo mineralIntakeServo;
    Servo mineralBoxServo;
    Servo mineralSlidesBlockServo;
    Servo markerKnockServo;
    Servo filterServo;
    DigitalChannel limitswitchwinch;
    DigitalChannel limitswitchmineralarm;
    double mil1startticks;
    double mil2startticks;
    double liftMotorstartticks;
    double liftdropticks;
    double mil1ticks;
    double mil2ticks;
    double prevmil1ticks;
    double mil1tickspersec;
    double boxmil1ticks;
    double variablespeed = 0;
    double inversespeed = 0;
    double mineralboxpos = 1;
    double speedmodifier = 1;
    boolean autoliftmode = false;
    boolean dropliftmmode = false;
    boolean xispressed = false;
    boolean yispressed = false;
    boolean intakeon = false;
    boolean intakereverse = false;
    boolean mineralarmendgame = false;
    boolean canuseautoliftmode = false;
    boolean boxScoop = false;
    boolean aPressed = false;
    boolean magnethit = false;
    boolean filternormal = true;
    double liftticks;
    int phase = 0;

    ElapsedTime mineralliftmodetimer = new ElapsedTime();
    ElapsedTime mineraldropmodetimer = new ElapsedTime();
    ElapsedTime mineralarmendgametimer = new ElapsedTime();
    ElapsedTime ytimer = new ElapsedTime();
    ElapsedTime xtimer = new ElapsedTime();
    ElapsedTime mil1tickpersectimer = new ElapsedTime();
    ElapsedTime silvermineraldroptimer = new ElapsedTime();
    ElapsedTime boxtimer = new ElapsedTime();
    ElapsedTime boxscooptimer = new ElapsedTime();
    ElapsedTime liftmotorstartticktimer = new ElapsedTime();
    ElapsedTime filtertimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        winchMotor = hardwareMap.dcMotor.get("wm");
        liftMotor = hardwareMap.dcMotor.get("lm");
        mineralIntakeServo = hardwareMap.crservo.get("mis");
        mineralBoxServo = hardwareMap.servo.get("mbs");
        mineralSlidesBlockServo = hardwareMap.servo.get("msbs");
        markerKnockServo = hardwareMap.servo.get("mks");
        filterServo = hardwareMap.servo.get("fs");
        mil1 = hardwareMap.dcMotor.get("mil1");
        mil2 = hardwareMap.dcMotor.get("mil2");
        limitswitchwinch = hardwareMap.digitalChannel.get("lsw");
        limitswitchmineralarm = hardwareMap.digitalChannel.get("lsma");

        mil1startticks = 0; //mil1.getCurrentPosition();
        mil2startticks = 0; //mil2.getCurrentPosition();
        liftMotorstartticks = 0; //liftMotor.getCurrentPosition();

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        mil2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mil1ticks = mil1startticks - mil1.getCurrentPosition();

        if (mil1ticks < GlobalVariables.MIL1_CLOSE_SLIDE_BLOCK_TICKS) {
            mineralSlidesBlockServo.setPosition(GlobalVariables.SLIDE_BLOCK_OPEN_POS);
        }

        markerKnockServo.setPosition(0);
    }

    @Override
    public void loop() {
        //Driving
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .2;
        double dpadturningpower = .4;
        // braking adjustment
        double brakingadjustmentticks = (GlobalVariables.LIFT_LOWERING_RETRACT_TICKS - GlobalVariables.LIFT_LOWERING_RETRACT_TICKS_BASE) / (GlobalVariables.LIFT_DROP_TICKS - GlobalVariables.LIFT_LOWERING_RETRACT_TICKS_BASE) * .06 * GlobalVariables.LIFT_DROP_TICKS;

        if (gamepad1.right_bumper) {
            speedmodifier = .5;
        }
        if (gamepad1.left_bumper) {
            speedmodifier = 1;
        }

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadturningpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadturningpower;
        } else {
            leftstickx = gamepad1.left_stick_x;
            leftsticky = -gamepad1.left_stick_y;
            rightstickx = gamepad1.right_stick_x * speedmodifier;
        }
        if (Math.abs(leftsticky) <= .15) {
            leftsticky = 0;
        }

        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5;
        leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
        rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
        leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
        rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

        leftFront.setPower(leftfrontpower);
        rightFront.setPower(rightfrontpower);
        leftRear.setPower(leftrearpower);
        rightRear.setPower(rightrearpower);

        //Mineral Intake
        if (gamepad2.right_bumper) {
            intakeon = true;
        }
        if (gamepad2.left_bumper) {
            intakeon = false;
        }

        if (intakeon) {
            if ((gamepad1.right_trigger == 1) || intakereverse) {
                mineralIntakeServo.setPower(GlobalVariables.MINERAL_INTAKE_REVERSE_SPEED);
            } else {
                mineralIntakeServo.setPower(GlobalVariables.MINERAL_INTAKE_FORWARD_SPEED);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                mineralIntakeServo.setPower(GlobalVariables.MINERAL_INTAKE_REVERSE_SPEED);
            } else {
                mineralIntakeServo.setPower(0);
            }
        }

        if (gamepad1.a && filtertimer.seconds() > .35){
            filternormal = !filternormal;
            filtertimer.reset();
        }

        if (filternormal){
            filterServo.setPosition(GlobalVariables.FILTER_SERVO_NORMAL);
            liftdropticks = GlobalVariables.LIFT_DROP_TICKS;
        } else {
            filterServo.setPosition(GlobalVariables.FILTER_SERVO_DEPOT);
            liftdropticks = GlobalVariables.LIFT_DEPOT_DROP_TICKS;
        }

        //Mineral Lift System
        mil1ticks = mil1startticks - mil1.getCurrentPosition();

        mil2ticks = mil2startticks - mil2.getCurrentPosition();

        liftticks = liftMotor.getCurrentPosition() - liftMotorstartticks;

        if (mil1tickpersectimer.seconds() > .05) {
            mil1tickspersec = (mil1ticks - prevmil1ticks) / mil1tickpersectimer.seconds();
            mil1tickpersectimer.reset();
            prevmil1ticks = mil1ticks;
        }

        if (gamepad2.y && ytimer.seconds() > .35) {
            yispressed = !yispressed;
            silvermineraldroptimer.reset();
            mineralboxpos = GlobalVariables.MINERAL_BOX_HALF_DROP;
            ytimer.reset();
        }

        if (gamepad2.x && xtimer.seconds() > .35) {
            xispressed = !xispressed;
            yispressed = false;
            xtimer.reset();
        }

        //Mineral Box
        if (yispressed) {
            if (silvermineraldroptimer.seconds() > GlobalVariables.MINERAL_BOX_HALFDROP_TURN_TIME + GlobalVariables.MINERAL_BOX_HALFDROP_PAUSE_TIME) {
                mineralboxpos = GlobalVariables.MINERAL_BOX_FULL_DROP;
            }
            xispressed = false;
        } else if (xispressed) {
            mineralboxpos = GlobalVariables.MINERAL_BOX_FULL_DROP;
            yispressed = false;
        } else {     // move the mineral box servo to angle that is based on the mineral arm angle (mil1ticks)
            boxmil1ticks = mil1ticks;
            if (boxmil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS + GlobalVariables.MIL1_TICKS_TO_TURN_BOX) {   // set the max for boxmilticks
                boxmil1ticks = GlobalVariables.MIL1_DROP_LEVEL_TICKS + GlobalVariables.MIL1_TICKS_TO_TURN_BOX;
            }
            if (boxmil1ticks < GlobalVariables.MIL1_DROP_LEVEL_TICKS) {                     // set the min for boxmilticks
                boxmil1ticks = GlobalVariables.MIL1_DROP_LEVEL_TICKS;
            }
            if (boxScoop && mil1ticks < GlobalVariables.MIL1_SCOOPING_TICKS) {
                mineralboxpos = GlobalVariables.MINERAL_BOX_BEGINSCOOP_POS;  // hold the box back to this position until arm is almost down
                boxscooptimer.reset();
            } else if (boxscooptimer.seconds() > .1) {
                if (autoliftmode || dropliftmmode || mineralarmendgame || mil1ticks <= GlobalVariables.MIL1_DROP_LEVEL_TICKS + 20) {
                    mineralboxpos = ((boxmil1ticks - GlobalVariables.MIL1_DROP_LEVEL_TICKS) / GlobalVariables.MIL1_TICKS_TO_TURN_BOX) * (GlobalVariables.MINERAL_BOX_BEGINLIFT_POS - GlobalVariables.MINERAL_BOX_LIFTED_POS) + GlobalVariables.MINERAL_BOX_LIFTED_POS;
                } else {
                    mineralboxpos = ((boxmil1ticks - GlobalVariables.MIL1_DROP_LEVEL_TICKS) / GlobalVariables.MIL1_TICKS_TO_TURN_BOX) * (GlobalVariables.MINERAL_BOX_INTAKE_POS - GlobalVariables.MINERAL_BOX_LIFTED_POS) + GlobalVariables.MINERAL_BOX_LIFTED_POS;
                }
            }
        }

        mineralBoxServo.setPosition(mineralboxpos);

        if (mil1ticks >= GlobalVariables.MIL1_CLOSE_SLIDE_BLOCK_TICKS) {
            mineralSlidesBlockServo.setPosition(GlobalVariables.SLIDE_BLOCK_CLOSED_POS);
            canuseautoliftmode = true;
            yispressed = false;
            xispressed = false;
        }

        aPressed = gamepad2.a;


        if (gamepad2.b && mineralliftmodetimer.seconds() >= .35) {
            autoliftmode = !autoliftmode;
            dropliftmmode = false;
            mineralarmendgame = false;
            boxScoop = false;
            intakereverse = false;
            magnethit = false;
            mineralliftmodetimer.reset();
            boxtimer.reset();
        }

        if (gamepad2.dpad_up && mineralarmendgametimer.seconds() >= .35) {
            mineralarmendgame = !mineralarmendgame;
            dropliftmmode = false;
            autoliftmode = false;
            boxScoop = false;
            intakereverse = false;
            mineralarmendgametimer.reset();
        }

        if ((gamepad2.dpad_down || aPressed) && mineraldropmodetimer.seconds() >= .35) {
            dropliftmmode = !dropliftmmode;
            autoliftmode = false;
            mineralarmendgame = false;
            mineraldropmodetimer.reset();
            boxScoop = aPressed;
            intakereverse = false;
        }

        if (gamepad2.dpad_left && liftmotorstartticktimer.seconds() > 1) {
            liftMotorstartticks = liftdropticks - liftticks;
            liftmotorstartticktimer.reset();
        }

        if (autoliftmode && canuseautoliftmode || mineralarmendgame) {

            intakereverse = false;

            if (!limitswitchmineralarm.getState()) {   // adjust start ticks
                liftMotorstartticks = liftdropticks - liftMotor.getCurrentPosition();
                liftticks = liftMotor.getCurrentPosition() - liftMotorstartticks;
            }

            if (mil1ticks > GlobalVariables.MIL1_RAISING_FULL_SPEED_TICKS) {
                phase = 1;
            } else if (mil1ticks > GlobalVariables.MIL1_DROP_LEVEL_TICKS) {
                phase = 2;
            } else {
                mineralarmendgame = false;
                phase = 3;
            }


            if (phase == 1) {
                if (boxtimer.seconds() <= .2) {  // don't start phase 1 until box has a chance to turn and we sweep backwards
                    intakereverse = true;
                } else {
                    if (boxtimer.seconds() <= .4) {  // don't start phase 1 until box has a chance to turn and we sweep backwards
                        intakereverse = true;
                    }
                    if (liftticks > GlobalVariables.LIFT_RAISING_RETRACT_TICKS) {  // retract mineral arm
                        liftMotor.setPower(-1);
                    } else {
                        liftMotor.setPower(0);
                    }
                    mil1.setPower(1);
                    mil2.setPower(1);
                }
            }

            if (phase == 2) {
                if (!mineralarmendgame) {
                    if (liftticks < liftdropticks) {  //extend mineral arm
                        liftMotor.setPower(1);
                    } else {
                        liftMotor.setPower(0);
                    }
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
                if (liftticks < liftdropticks) {
                    liftMotor.setPower(1);  //extend mineral arm
                } else {
                    liftMotor.setPower(0);
                    autoliftmode = false;
                }
            }
        } else if (dropliftmmode) {

            if (mil1ticks < GlobalVariables.MIL1_FULL_SPEED_DEPLOY_TICKS) {
                mil1.setPower(-1);
                mil2.setPower(-1);
            } else if (mil1ticks < GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_TICKS) {
                mil1.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
                mil2.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
                if (liftticks > GlobalVariables.LIFT_LOWERING_RETRACT_TICKS) {
                    liftMotor.setPower(-1);    // retract mineral arm
                } else {
                    liftMotor.setPower(0);
                }
            } else if (mil1ticks < GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS) {
                if (liftticks > GlobalVariables.LIFT_LOWERING_RETRACT_TICKS) {
                    liftMotor.setPower(-1);   // retract mineral arm
                } else {
                    liftMotor.setPower(0);
                }
                if (mil1tickspersec > GlobalVariables.MIL1_MAX_TICKS_PER_SECOND) {
                    mil1.setPower(0);  // braking
                    mil2.setPower(0);
                } else {
                    mil1.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                    mil2.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                }
            } else {  // braking
                mil1.setPower(0);
                mil2.setPower(0);
                dropliftmmode = false;
                boxScoop = false;
            }
/*
            if (liftticks > GlobalVariables.LIFT_LOWERING_RETRACT_TICKS) {
                liftMotor.setPower(-1);    // retract mineral arm
            } else {
                liftMotor.setPower(0);
            }
            if (mil1ticks < GlobalVariables.MIL1_FULL_SPEED_DEPLOY_TICKS) {
                mil1.setPower(-1);
                mil2.setPower(-1);
            } else if (mil1ticks < GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_TICKS) {
                mil1.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
                mil2.setPower(GlobalVariables.MIL1_MEDIUM_SPEED_DEPLOY_POWER);
            } else if (mil1ticks < GlobalVariables.MIL1_BRAKING_DEPLOY_TICKS - brakingadjustmentticks) {
                if (mil1tickspersec > GlobalVariables.MIL1_MAX_TICKS_PER_SECOND) {
                    mil1.setPower(0);  // braking
                    mil2.setPower(0);
                } else {
                    mil1.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                    mil2.setPower(GlobalVariables.MIL1_LOW_SPEED_DEPLOY_POWER);
                }
                /*
                variablespeed = 1 - (((double)mil1ticks / GlobalVariables.MAX_MIL1_TICKS - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE + (1 - GlobalVariables.MIL1_BRAKING_DEPLOY_PERCENTAGE)) / (1 - GlobalVariables.MIL1_FULL_SPEED_DEPLOY_PERCENTAGE));
                if (variablespeed < 0) {
                    variablespeed = 0;
                }
                inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND - mil1tickspersec;
                if (inversespeed < GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100) {
                    inversespeed = GlobalVariables.MIL1_MAX_TICKS_PER_SECOND/100;
                }
                inversespeed = inversespeed / GlobalVariables.MIL1_MAX_TICKS_PER_SECOND;
                mil1.setPower(-variablespeed);
                mil2.setPower(-variablespeed);

            } else {  // braking
                if (mil1ticks < GlobalVariables.MIL1_SCOOPING_TICKS) {
                    if (mil1tickspersec > 300) {
                        mil1.setPower(0);
                        mil2.setPower(0);
                    } else {
                        mil1.setPower(-.1);  // keep it moving until scooping ticks achieved
                        mil2.setPower(-.1);
                    }
                } else {
                    mil1.setPower(0);
                    mil2.setPower(0);
                    liftMotor.setPower(0);
                    dropliftmmode = false;
                    boxScoop = false;
                }
            }
*/
            yispressed = false;
            xispressed = false;

        } else {
            mil1.setPower(gamepad2.right_stick_y);
            mil2.setPower(gamepad2.right_stick_y);
            liftMotor.setPower(-gamepad2.left_stick_y);
        }


        //Winch Lift
        boolean magnetistouching;

        magnetistouching = !limitswitchwinch.getState();

        if (gamepad1.left_trigger >= .75 && !magnetistouching || gamepad2.right_trigger >= .75 && !magnetistouching) {
            winchMotor.setPower(1);
        } else if (gamepad2.left_trigger >= .75) {
            winchMotor.setPower(-1);
        } else {
            winchMotor.setPower(0);
        }

        telemetry.addData("Mil 1 Ticks", mil1ticks);
        telemetry.addData("Mil 1 Ticks Percent", (double) mil1ticks / GlobalVariables.MAX_MIL1_TICKS);
        telemetry.addData("Mil 2 Ticks Percent", (double) mil2ticks / GlobalVariables.MAX_MIL1_TICKS);
        telemetry.addData("Drop Level Ticks", GlobalVariables.MIL1_DROP_LEVEL_TICKS);
        telemetry.addData("Lift Ticks", liftticks);
        telemetry.addData("Lift Start Ticks", liftMotorstartticks);
        telemetry.addData("Box Pos", mineralboxpos);
        telemetry.addData("Ticks per Second", mil1tickspersec);
        telemetry.addData("Variable Speed", variablespeed);
        telemetry.addData("Inverse Speed Adjustment", inversespeed);
        telemetry.addData("Prev mil1 Ticks", prevmil1ticks);
        telemetry.addData("Phase", phase);
        telemetry.addData("mil1 Start Ticks", mil1startticks);
        telemetry.addData("Magnet Hit", magnethit);
        telemetry.update();

    }
}
