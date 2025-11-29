package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DemoBotServo {
    private Servo endaffector;

    private CRServo servoRot;

    public void init(HardwareMap hwMap){
        endaffector = hwMap.get(Servo.class, "end_affector");
    }
    public void setEndaffectorPos(double angle){
        endaffector.setPosition(angle);
    }
}
