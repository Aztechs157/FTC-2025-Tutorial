package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lessCowbellIntake {
    private DcMotor intake;


    public void init(HardwareMap hwMap){
        //create motors
        intake = hwMap.get(DcMotor.class, "intake");

        //set mode
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set direction
        intake.setDirection(DcMotor.Direction.FORWARD);





    }
    //setter function
    public void setIntakeSpeed(double speed){
        intake.setPower(speed);
    }
}
