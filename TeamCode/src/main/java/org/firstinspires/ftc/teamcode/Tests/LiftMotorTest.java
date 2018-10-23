
package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp (name = "Lift Motor Tests", group = "Tests")
public class LiftMotorTest extends OpMode {

    DcMotor liftmotor;
    DcMotor liftmotor2;

    @Override
    public void init() {
        liftmotor = hardwareMap.dcMotor.get("lm");
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor2.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        liftmotor.setPower(gamepad1.left_stick_y);
    }
}
