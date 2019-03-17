package org.firstinspires.ftc.teamcode;

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
    DigitalChannel limitswitchwinch;
    DigitalChannel limitswitchmineralarm;
    int mil1startticks;
    int mil2startticks;
    int liftMotorstartticks;
    int prevmil1ticks;
    double mil1tickspersec;
    double mineralboxpos = 1;
    double speedmodifier = 1;
    double boxboost = 0;
    boolean autoliftmode = false;
    boolean dropliftmmode = false;
    boolean xispressed = false;
    boolean yispressed = false;
    boolean intakeon = false;
    boolean mineralarmendgame = false;
    boolean canuseautoliftmode = false;
    boolean boxScoop = false;
    boolean dpadDownPressed = false;
    int mil1ticks;

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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mil1ticks = mil1startticks - mil1.getCurrentPosition();

        if (mil1ticks >= 300) {

        } else {
            mineralSlidesBlockServo.setPosition(.5);
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
            if (gamepad1.right_trigger == 1) {
                mineralIntakeServo.setPower(.88);
            } else {
                mineralIntakeServo.setPower(-.88);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                mineralIntakeServo.setPower(.88);
            } else {
                mineralIntakeServo.setPower(0);
            }
        }


        //Mineral Lift System
        int mil2ticks;
        int liftticks;
        int phase = 0;
        final double servoturnpos = .16; // the pos where the servo starts turning
        final double maxmil1ticks = GlobalVariables.MAX_MIL1_TICKS;
        final double droplevelticks = 70;
        final int liftdropticks = 1150;
        final double liftlowerticks = liftdropticks / 4;
        final double tickstoturnbox = maxmil1ticks - droplevelticks; // box must be fully turned between droplevelticks and droplevelticks + tickstoturnbox
        double boxmil1ticks;

        mil1ticks = mil1startticks - mil1.getCurrentPosition();

        mil2ticks = mil2startticks - mil2.getCurrentPosition();

        liftticks = liftMotorstartticks - liftMotor.getCurrentPosition();

        if (mil1tickpersectimer.seconds() > .05) {
            mil1tickspersec = (mil1ticks - prevmil1ticks) / mil1tickpersectimer.seconds();
            mil1tickpersectimer.reset();
        } else {
            prevmil1ticks = mil1ticks;
        }


        if (gamepad2.y && ytimer.seconds() > .35) {
            yispressed = !yispressed;
            silvermineraldroptimer.reset();
            mineralboxpos = .43;
            ytimer.reset();
        }

        if (gamepad2.x && xtimer.seconds() > .35) {
            xispressed = !xispressed;
            yispressed = false;
            xtimer.reset();
        }

        //Mineral Box
        if (yispressed) {
            // mineralboxpos = .43;
            if (silvermineraldroptimer.seconds() > .75) {
                mineralboxpos = GlobalVariables.MINERAL_BOX_FULL_DROP;
            }
            xispressed = false;
        } else if (xispressed) {
            mineralboxpos = GlobalVariables.MINERAL_BOX_FULL_DROP;
            yispressed = false;
        } else {     // move the mineral box servo to angle that is based on the mineral arm angle (mil1ticks)
            /*
            if(autoliftmode){
                minrangedifference = .55;  // tip the box up a bit more if we're trying to lift
            } else {
                minrangedifference = .36;
            }
            */
            boxmil1ticks = mil1ticks;
            if (boxmil1ticks > droplevelticks + tickstoturnbox) {   // set the max for boxmilticks
                boxmil1ticks = droplevelticks + tickstoturnbox;
            }
            if (boxmil1ticks < droplevelticks) {                     // set the min for boxmilticks
                boxmil1ticks = droplevelticks;
            }
            if (boxScoop && mil1ticks < 465) {
                mineralboxpos = .25;
                boxscooptimer.reset();
            } else if (boxscooptimer.seconds() < .1) {
                mineralboxpos = .25;
            } else {
                mineralboxpos = ((boxmil1ticks - droplevelticks) / tickstoturnbox) * (GlobalVariables.MINERAL_BOX_GROUND_LEVEL - servoturnpos) + servoturnpos + boxboost; // makes a range from .2 to .51

            }
        }

        mineralBoxServo.setPosition(mineralboxpos);

        if (mil1ticks >= 300) {
            mineralSlidesBlockServo.setPosition(.96);
            canuseautoliftmode = true;
            yispressed = false;
            xispressed = false;
        }

        dpadDownPressed = gamepad2.a;

        if (gamepad2.b && mineralliftmodetimer.seconds() >= .35) {
            autoliftmode = !autoliftmode;
            dropliftmmode = false;
            mineralarmendgame = false;
            boxScoop = false;
            mineralliftmodetimer.reset();
            boxtimer.reset();
        }

        if ((gamepad2.dpad_down || dpadDownPressed) && mineraldropmodetimer.seconds() >= .35) {
            dropliftmmode = !dropliftmmode;
            autoliftmode = false;
            mineralarmendgame = false;
            mineraldropmodetimer.reset();
            boxScoop = dpadDownPressed;
        }

        if (gamepad2.dpad_left && liftmotorstartticktimer.seconds() > 1) {
            liftMotorstartticks = liftdropticks - liftticks;
            liftmotorstartticktimer.reset();
        }

        if (gamepad2.dpad_up && mineralarmendgametimer.seconds() >= .35) {
            mineralarmendgame = !mineralarmendgame;
            dropliftmmode = false;
            autoliftmode = false;
            boxScoop = false;
            mineralarmendgametimer.reset();
        }

        if (autoliftmode && canuseautoliftmode || mineralarmendgame) {
            if (boxtimer.seconds() < .2) {
                boxboost = .05;
            } else if (mil1ticks > 225) {
                phase = 1;
            } else if (mil1ticks > droplevelticks) {
                phase = 2;
                boxboost = 0;
            } else {
                mineralarmendgame = false;
                phase = 3;
            }

            if (phase == 1) {
                if (liftticks > liftlowerticks) {
                    liftMotor.setPower(1);
                } else {
                    liftMotor.setPower(0);
                }
                mil1.setPower(1);
                mil2.setPower(1);
                 /*else {
                    mil1.setPower(.65);
                    mil2.setPower(.65);
                }
                */
            }

            if (phase == 2) {
                if (!mineralarmendgame) {
                    if (limitswitchmineralarm.getState()) {
                        liftMotor.setPower(-1);
                    } else {
                        liftMotor.setPower(0);
                    }
                }
                if (mil1ticks > 130) {
                    mil1.setPower(.4);
                    mil2.setPower(.4);
                } else if (mil1ticks > droplevelticks + 25) {
                    mil1.setPower(.1);
                    mil2.setPower(.1);
                } else {
                    mil1.setPower(.3);
                    mil2.setPower(.3);
                }
            }

            if (phase == 3) {
                /*
                if (mil1ticks < droplevelticks){
                    mil1.setPower(-.15);
                    mil2.setPower(-.15);
                } else {
                    mil1.setPower(0);
                    mil2.setPower(0);
                }
*/
                mil1.setPower(0);
                mil2.setPower(0);
                if (limitswitchmineralarm.getState()) {
                    liftMotor.setPower(-1);
                } else {
                    liftMotor.setPower(0);
                    liftMotorstartticks = liftdropticks - liftticks;
                    autoliftmode = false;
                }
                /*
                if (liftticks < liftdropticks) {
                    liftMotor.setPower(-1);
                } else {
                    liftMotor.setPower(0);
                    //mil1startticks = (int)(mil1startticks + droplevelticks - mil1ticks);
                }
                */
            }
        } else if (dropliftmmode) {
            if (mil1ticks < 120) {
                mil1.setPower(-1);
                mil2.setPower(-1);
            } else if (mil1ticks < 390) {
                mil1.setPower(-.75);
                mil2.setPower(-.75);
                if (liftticks > liftlowerticks) {
                    liftMotor.setPower(1);
                } else {
                    liftMotor.setPower(0);
                }
            } else if (mil1ticks < 420) {
                if (liftticks > liftlowerticks) {
                    liftMotor.setPower(1);
                } else {
                    liftMotor.setPower(0);
                }
                if (mil1tickspersec > 700 && liftticks > liftlowerticks) {
                    mil1.setPower(0);
                    mil2.setPower(0);
                } else {
                    mil1.setPower(-.25);
                    mil2.setPower(-.25);
                }
            } else {
                mil1.setPower(0);
                mil2.setPower(0);
                dropliftmmode = false;
                boxScoop = false;
            }

            yispressed = false;
            xispressed = false;
            boxboost = 0;

        } else {
            mil1.setPower(gamepad2.right_stick_y);
            mil2.setPower(gamepad2.right_stick_y);
            if (mil1ticks <= droplevelticks) {
                liftMotor.setPower(gamepad2.left_stick_y / 2);
            } else {
                liftMotor.setPower(gamepad2.left_stick_y);
            }
            boxboost = 0;
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
        telemetry.addData("Mil 2 Ticks", mil2ticks);
        telemetry.addData("Lift Motor Ticks", liftticks);
        telemetry.addData("Box Pos", mineralboxpos);
        telemetry.addData("Ticks per Second", mil1tickspersec);
        telemetry.addData("Prev mil1 Ticks", prevmil1ticks);
        telemetry.addData("Phase", phase);
        telemetry.addData("mil1 Start Ticks", mil1startticks);
        telemetry.update();

    }
}
