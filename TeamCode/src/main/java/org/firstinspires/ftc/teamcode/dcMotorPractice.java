package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DemoBot;

@TeleOp
public class dcMotorPractice extends OpMode {
    DemoBot bot = new DemoBot();
    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        bot.setLeftMotorSpeed(0.1);

    }
}
