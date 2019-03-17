package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "RPMTest", group = "Tests")
public class RPMTest extends OpMode {

    CRServo VEXservo;
    double servoPower;

    ElapsedTime buttontimer = new ElapsedTime();

    @Override
    public void init() {
    VEXservo = hardwareMap.crservo.get("mis");
    }

    @Override
    public void loop() {
        if (gamepad1.y && buttontimer.seconds() > .1){
            servoPower = servoPower + 0.01;
            buttontimer.reset();
        }

        if (gamepad1.a && buttontimer.seconds() > .1){
            servoPower = servoPower - 0.01;
            buttontimer.reset();
        }

        VEXservo.setPower(servoPower);

        telemetry.addData("Power", servoPower);
    }
}
