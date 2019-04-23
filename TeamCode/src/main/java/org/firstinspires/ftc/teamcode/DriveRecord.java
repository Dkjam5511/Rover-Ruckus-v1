package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Scanner;


@TeleOp(name = "Drive Record", group = "TeleOp")
public class DriveRecord extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    public static final int motorpowersarraysize = 100000;
    public static final String motorpowerinputfile = Environment.getExternalStorageDirectory() + "/motorpowersAUTON.txt";

    int motorpowerindex = 0;
    double[] motorpowers = new double[motorpowersarraysize];
    int filenumber = 1;

    boolean xispressed = false;
    boolean aispressed = false;


    ElapsedTime ytimer = new ElapsedTime();
    ElapsedTime xtimer = new ElapsedTime();
    ElapsedTime atimer = new ElapsedTime();
    ElapsedTime recordTimer = new ElapsedTime();
    ElapsedTime playTimer = new ElapsedTime();


    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        File mpinputfile = new File(motorpowerinputfile);
        if (mpinputfile.exists()) {
            motorpowers = readMP();
        }
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


        if (gamepad1.y && ytimer.seconds() > .35) {
            if (xispressed) {
                motorpowers[0] = motorpowerindex;
                storeMP(motorpowers, filenumber);
                filenumber = filenumber + 1;
                xispressed = false;
                motorpowerindex = 0;
            }
            ytimer.reset();
        }

        if (gamepad1.x && xtimer.seconds() > .35) {
            xispressed = true;
            motorpowerindex = 0;
            aispressed = false;
            recordTimer.reset();
            xtimer.reset();
        }

        if (gamepad1.a && atimer.seconds() > .35 && !xispressed) {
            atimer.reset();
            playTimer.reset();
            aispressed = !aispressed;
        }


        if (aispressed) { //PLAYING
            motorpowerindex = ((int) (playTimer.seconds() * 50)) * 4;
            leftfrontpower = motorpowers[motorpowerindex + 1];
            rightfrontpower = motorpowers[motorpowerindex + 2];
            leftrearpower = motorpowers[motorpowerindex + 3];
            rightrearpower = motorpowers[motorpowerindex + 4];
            if (motorpowerindex >= motorpowers[0]) {
                aispressed = false;
            }
        } else { //RECORDING
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
            leftfrontpower = (wheelpower * Math.cos(stickangleradians) + rightX);
            rightfrontpower = (wheelpower * Math.sin(stickangleradians) - rightX);
            leftrearpower = (wheelpower * Math.sin(stickangleradians) + rightX);
            rightrearpower = (wheelpower * Math.cos(stickangleradians) - rightX);
        }

        leftFront.setPower(leftfrontpower);
        rightFront.setPower(rightfrontpower);
        leftRear.setPower(leftrearpower);
        rightRear.setPower(rightrearpower);

        if (xispressed) {
            motorpowerindex = ((int) (recordTimer.seconds() * 50)) * 4;
            motorpowers[motorpowerindex + 1] = (leftfrontpower);
            motorpowers[motorpowerindex + 2] = (rightfrontpower);
            motorpowers[motorpowerindex + 3] = (leftrearpower);
            motorpowers[motorpowerindex + 4] = (rightrearpower);
            if (motorpowerindex >= motorpowersarraysize - 4) {
                xispressed = false;
            }
        }

        telemetry.addData("Recording", xispressed);
        telemetry.addData("File Number", filenumber);
        telemetry.addData("Motor Power Index", motorpowerindex);
        telemetry.addData("Motor Power Entries", motorpowers[0]);
        telemetry.update();

    }

    private static void storeMP(double[] motorpowers, int filenumber) throws RuntimeException {
        String filename = Environment.getExternalStorageDirectory() + "/motorpowers" + Integer.toString(filenumber) + ".txt";
        String newLine = System.getProperty("line.separator");
        DataOutputStream dos;
        try {
            FileOutputStream fos = new FileOutputStream(filename);
            dos = new DataOutputStream(fos);
            for (int i = 0; i < motorpowers[0] + 5; i++) {
                dos.writeBytes(Double.toString(motorpowers[i]) + newLine);
            }
            dos.close();
        } catch (IOException e) {
            System.out.println("IOException : " + e);
        }
    }

    private static double[] readMP() {
        double[] motorpowers = new double[motorpowersarraysize];
        int i = 0;
        try {
            Scanner inputfile = new Scanner(new File(motorpowerinputfile));
            while (inputfile.hasNext()) {
                motorpowers[i] = inputfile.nextDouble();
                i = i + 1;
            }
        } catch (IOException e) {
            System.out.println("IOException : " + e);
        }
        return motorpowers;
    }

}
