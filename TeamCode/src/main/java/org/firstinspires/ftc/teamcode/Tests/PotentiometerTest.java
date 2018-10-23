
package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp (name = "Potentiometer Test", group = "Tests")
public class PotentiometerTest extends OpMode {

    AnalogInput potentiometer;

    @Override
    public void init() {
        potentiometer = hardwareMap.analogInput.get("pr");
    }

    @Override
    public void loop() {
        telemetry.addData("Voltage", potentiometer.getVoltage());
        telemetry.addData("Max VOltage", potentiometer.getMaxVoltage());

    }
}
