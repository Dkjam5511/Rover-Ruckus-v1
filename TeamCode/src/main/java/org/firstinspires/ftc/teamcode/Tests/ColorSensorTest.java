package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test", group = "Tests")
public class ColorSensorTest extends OpMode {

    ColorSensor cs;
    ColorSensor cs2;
    ColorSensor frontcs;
    DistanceSensor csd;
    DistanceSensor csd2;

    @Override
    public void init() {
        cs = hardwareMap.colorSensor.get("mcs");
        csd = hardwareMap.get(DistanceSensor.class, "mcs");
        cs2 = hardwareMap.colorSensor.get("mcs2");
        csd2 = hardwareMap.get(DistanceSensor.class, "mcs2");
        frontcs = hardwareMap.colorSensor.get("fcs");
    }

    @Override
    public void loop() {

        boolean yellowfound ;
        double colorval;

        if (cs.alpha() > cs2.alpha()) {
            colorval = (cs.red() - cs.blue());
        } else {
            colorval = (cs2.red() - cs2.blue());
        }
        if (colorval >= 10) {
            yellowfound = true;
        } else {
            yellowfound = false;
        }
        telemetry.addData("YELLOW", yellowfound);
        telemetry.addData("Red", cs.red());
        telemetry.addData("Blue", cs.blue());
        telemetry.addData("Green", cs.green());
        telemetry.addData("Alpha", cs.alpha());
        telemetry.addData("Distance", csd.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red2", cs2.red());
        telemetry.addData("Blue2", cs2.blue());
        telemetry.addData("Green2", cs2.green());
        telemetry.addData("Alpha2", cs2.alpha());
        telemetry.addData("Distance2", csd2.getDistance(DistanceUnit.INCH));
        telemetry.addData("RedFront", frontcs.red());
        telemetry.addData("BlueFront", frontcs.blue());
        telemetry.addData("GreenFront", frontcs.green());
        telemetry.addData("AlphaFront", frontcs.alpha());
        telemetry.update();
    }
}
