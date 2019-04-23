package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Filer Servo Test", group = "Tests")
public class FilterServoTest extends OpMode {

    Servo filterServo;
    double servopos = .5;
    ElapsedTime TIMER = new ElapsedTime();

    @Override
    public void init() {
        filterServo = hardwareMap.servo.get("fs");
    }

    @Override
    public void loop() {
        if (gamepad1.y && TIMER.seconds() > .05){
            servopos = servopos + .01;
            TIMER.reset();
        }

        if (gamepad1.a && TIMER.seconds() > .05){
            servopos = servopos - .01;
            TIMER.reset();
        }

        filterServo.setPosition(servopos);
        telemetry.addData("Servo Pos", servopos);
    }
}
