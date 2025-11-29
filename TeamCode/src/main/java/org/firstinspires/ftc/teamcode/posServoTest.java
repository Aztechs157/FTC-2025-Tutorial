package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DemoBotServo;

@TeleOp
public class posServoTest extends OpMode {
    DemoBotServo bot = new DemoBotServo();
    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.setEndaffectorPos(0);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            bot.setEndaffectorPos(-1);
        }else {
            bot.setEndaffectorPos(1);
        }
    }
}
