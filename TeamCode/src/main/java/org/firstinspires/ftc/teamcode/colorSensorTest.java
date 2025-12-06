package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.SpindexerSensor;
@TeleOp

public class colorSensorTest extends OpMode {
    SpindexerSensor sensor;

    @Override
    public void init() {
        sensor = new SpindexerSensor(hardwareMap, "colorSensor3-intake", telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("Intake Sensor State: ", sensor.getDetectedColor());
    }
}
