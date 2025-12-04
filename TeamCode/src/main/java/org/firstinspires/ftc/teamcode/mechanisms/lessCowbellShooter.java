package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lessCowbellShooter {
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor hopper;
    public void init(HardwareMap hwMap){

        //create motors
        hopper = hwMap.get(DcMotor.class, "hopper");
        shooterLeft = hwMap.get(DcMotor.class, "shooter_left");
        shooterRight = hwMap.get(DcMotor.class,"shooter_right");

        // set mode
        hopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set directions
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
    }
    // setter functions
    public void setHopperSpeed (double speed){
        hopper.setPower(speed);
    }
    public void setShooterSpeed (double speed){
        shooterLeft.setPower(speed);
        shooterRight.setPower(speed);
    }
}
