package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DcMotor winchmotor;
    DcMotor mil1;
    DcMotor mil2;
    DcMotor liftmotor;
    Servo mineralknockservo;
    Servo mineralintakeservo;
    Servo mineralboxservo;
    Servo mineralslidesblockservo;
    DigitalChannel magneticlimitswitch;
    int mil1startticks;
    int mil2startticks;
    int liftmotorstartticks;
    boolean autoliftmode = false;
    boolean dropliftmmode = false;
    boolean yispressed = false;
    boolean xispressed = false;
    boolean intakeon = false;
    boolean mineralarmendgame = false;

    ElapsedTime mineralliftmodetimer = new ElapsedTime();
    ElapsedTime mineraldropmodetimer = new ElapsedTime();
    ElapsedTime mineralarmendgametimer = new ElapsedTime();
    ElapsedTime ytimer = new ElapsedTime();
    ElapsedTime xtimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        winchmotor = hardwareMap.dcMotor.get("wm");
        liftmotor = hardwareMap.dcMotor.get("lm");
        mineralknockservo = hardwareMap.servo.get("mks");
        mineralintakeservo = hardwareMap.servo.get("mis");
        mineralboxservo = hardwareMap.servo.get("mbs");
        mineralslidesblockservo = hardwareMap.servo.get("msbs");
        mil1 = hardwareMap.dcMotor.get("mil1");
        mil2 = hardwareMap.dcMotor.get("mil2");
        magneticlimitswitch = hardwareMap.digitalChannel.get("mls");

        mineralknockservo.setPosition(1);
        mineralslidesblockservo.setPosition(0);

        mil1startticks = mil1.getCurrentPosition();
        mil2startticks = mil2.getCurrentPosition();
        liftmotorstartticks = liftmotor.getCurrentPosition();

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        winchmotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mil2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        double speedmodifier = 1;

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
            leftstickx = gamepad1.left_stick_x * speedmodifier;
            leftsticky = -gamepad1.left_stick_y * speedmodifier;
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

        //Mineral Intake and box
        if (gamepad2.right_bumper) {
            intakeon = true;
        }
        if (gamepad2.left_bumper) {
            intakeon = false;
        }

        if (intakeon) {
            if (gamepad1.right_trigger == 1) {
                mineralintakeservo.setPosition(0);
            } else {
                mineralintakeservo.setPosition(1);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                mineralintakeservo.setPosition(0);
            } else {
                mineralintakeservo.setPosition(.5);
            }
        }


        //Mineral Lift System
        int mil1ticks;
        int mil2ticks;
        int liftticks;
        int phase = 0;
        double servoturnpos = .2;
        double mineralboxpos;
        double maxmil1ticks = 351;
        double droplevelticks = 80;
        double tickstoturnbox = 200; // box must be fully turned between droplevelticks and droplevelticks + tickstoturnbox
        double boxmil1ticks;

        mil1ticks = mil1startticks - mil1.getCurrentPosition();
        mil2ticks = mil2startticks - mil2.getCurrentPosition();
        liftticks = liftmotorstartticks - liftmotor.getCurrentPosition();

        boxmil1ticks = mil1ticks;

        if (boxmil1ticks > droplevelticks + tickstoturnbox) {
            boxmil1ticks = droplevelticks + tickstoturnbox;
        }
        if (boxmil1ticks < droplevelticks) {
            boxmil1ticks = droplevelticks;
        }

        if (gamepad2.y && ytimer.seconds() > .35) {
            yispressed = !yispressed;
            xispressed = false;
            ytimer.reset();
        }

        if (gamepad2.x && xtimer.seconds() > .35) {
            xispressed = !xispressed;
            yispressed = false;
            xtimer.reset();
        }

        if (yispressed) {
            mineralboxpos = .5;
        } else if (xispressed) {
            mineralboxpos = .35;
        } else {
            mineralboxpos = ((boxmil1ticks - droplevelticks) / tickstoturnbox) * (.5 - servoturnpos) + servoturnpos; // makes a range from .2 to .5
        }

        if (mil1ticks > 200) {
            yispressed = false;
            xispressed = false;
        }

        mineralboxservo.setPosition(mineralboxpos);

        if (gamepad2.b && mineralliftmodetimer.seconds() >= .35) {
            autoliftmode = !autoliftmode;
            dropliftmmode = false;
            mineralliftmodetimer.reset();
        }

        if (gamepad2.a && mineraldropmodetimer.seconds() >= .35) {
            dropliftmmode = !dropliftmmode;
            autoliftmode = false;
            mineraldropmodetimer.reset();
        }

        if (gamepad2.dpad_up && mineralarmendgametimer.seconds() >= .35) {
            mineralarmendgame = !mineralarmendgame;
            mineralarmendgametimer.reset();
        }

        if (autoliftmode || mineralarmendgame) {
            if (mil1ticks > 180) {
                phase = 1;
            } else if (mil1ticks > droplevelticks) {
                phase = 2;
            } else {
                mineralarmendgame = false;
                phase = 3;
            }

            if (phase == 1) {
                if (liftticks > 100) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                mil1.setPower(.65);
                mil2.setPower(.65);
            }

            if (phase == 2) {
                if (liftticks > 100) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                mil1.setPower(.3);
                mil2.setPower(.3);
            }

            if (phase == 3) {
                if (!mineralarmendgame) {
                    mil1.setPower(0);
                    mil2.setPower(0);
                    if (liftticks < 4000) {
                        liftmotor.setPower(-1);
                    } else {
                        liftmotor.setPower(0);
                        autoliftmode = false;
                    }
                }
            }
        } else if (dropliftmmode) {
            if (mil1ticks < 150) {
                mil1.setPower(-.4);
                mil2.setPower(-.4);
            } else if (mil1ticks < 250) {
                mil1.setPower(-.2);
                mil2.setPower(-.2);
            } else {
                mil1.setPower(0);
                mil2.setPower(0);
                dropliftmmode = false;
            }

            yispressed = false;
            xispressed = false;

            if (liftticks > 1500) {
                liftmotor.setPower(1);
            } else {
                liftmotor.setPower(0);
            }

        } else {
            if (gamepad2.right_stick_y <= 0 && mil1ticks > maxmil1ticks) {
                mil1.setPower(.3);
                mil2.setPower(.3);
            } else {
                mil1.setPower(gamepad2.right_stick_y / 2);
                mil2.setPower(gamepad2.right_stick_y / 2);
            }
            liftmotor.setPower(gamepad2.left_stick_y);
        }

        if (mil1ticks >= 300) {
            mineralslidesblockservo.setPosition(.5);
        }


        //Winch Lift
        boolean magnetistouching;

        magnetistouching = !magneticlimitswitch.getState();

        if (gamepad2.right_trigger >= .75 && !magnetistouching) {
            winchmotor.setPower(1);
        } else if (gamepad2.left_trigger >= .75) {
            winchmotor.setPower(-1);
        } else {
            winchmotor.setPower(0);
        }

        telemetry.addData("Mil 1 Ticks", mil1ticks);
        telemetry.addData("Mil 2 Ticks", mil2ticks);
        telemetry.addData("Lift Motor Ticks", liftticks);
        telemetry.addData("Auto Lift Mode", autoliftmode);
        telemetry.addData("Box Pos", mineralboxpos);
        telemetry.addData("mineralarmendgame", mineralarmendgame);
        telemetry.addData("mineralarmendgame",mineralarmendgametimer.seconds());
        telemetry.addData("Phase", phase);
        telemetry.update();

    }
}
