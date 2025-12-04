package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lessCowbellSpindexer {
    private CRServo spindexer;

    public void init(HardwareMap hwmap){
        //create motor
        spindexer = hwmap.get(CRServo.class, "spindexer");
        //set direction
        spindexer.setDirection(DcMotor.Direction.FORWARD);
        //set mode
    }
    public void setSpindexerSpeed(double speed){
        spindexer.setPower(speed);
    }

}
