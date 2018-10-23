package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp (name = "Magnetic Limit Switch Test", group = "Tests")
public class MagneticLimitSwitchTest extends OpMode {

    DigitalChannel magneticlimitswitch;

    @Override
    public void init() {
        magneticlimitswitch = hardwareMap.digitalChannel.get("mls");
    }

    @Override
    public void loop() {

        magneticlimitswitch.setMode(DigitalChannel.Mode.INPUT);

        if (magneticlimitswitch.getState() == true){
            telemetry.addLine("OFF");
        } else {
            telemetry.addLine("ON");
        }

    }
}
