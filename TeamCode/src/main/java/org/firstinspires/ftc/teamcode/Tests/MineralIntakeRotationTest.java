package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MI Rotation Test", group = "Tests")
public class MineralIntakeRotationTest extends OpMode {

    DcMotor mil1;
    DcMotor mil2;

    int mil1startticks;
    int mil2startticks;

    @Override
    public void init() {
        mil1 = hardwareMap.dcMotor.get("mil1");
        mil2 = hardwareMap.dcMotor.get("mil2");

        mil1startticks = mil1.getCurrentPosition();
        mil2startticks = mil2.getCurrentPosition();
    }

    @Override
    public void loop() {
        int mil1ticks;
        int mil2ticks;

        mil1ticks = mil1startticks - mil1.getCurrentPosition();
        mil2ticks = mil2startticks - mil2.getCurrentPosition();

        telemetry.addData("mil1Ticks", mil1ticks);
        telemetry.addData("mil2Ticks", mil2ticks);
    }
}
