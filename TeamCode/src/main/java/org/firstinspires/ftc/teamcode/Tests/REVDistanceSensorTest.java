package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "REV Distance Sensor Test", group = "Tests")
public class REVDistanceSensorTest extends OpMode {

    DistanceSensor distancesensor;
    double distance;

    @Override
    public void init() {
        distancesensor = hardwareMap.get(DistanceSensor.class, "ds");

    }
    @Override
    public void loop() {
        distance = distancesensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance", distance);

    }
}
