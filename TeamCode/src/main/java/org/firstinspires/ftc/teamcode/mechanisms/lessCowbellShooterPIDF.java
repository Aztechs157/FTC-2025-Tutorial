package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class lessCowbellShooterPIDF {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotor hopper;

    public double highVelocity = 2200;
    public double lowVelocity = 1500;
    public double curTargetVelocty = lowVelocity;

    public double F = 10; // 10
     public double P = 30; // 30

    public double[] stepSizes ={10.0, 1.0, 0.1, 0.801, 0.0801};

    public int stepIndex =1;


    public void init(HardwareMap hwMap){

        //create motors
        hopper = hwMap.get(DcMotor.class, "hopper");
        shooterLeft = hwMap.get(DcMotorEx.class, "shooter_left");
        shooterRight = hwMap.get(DcMotorEx.class,"shooter_right");

        // set mode
        hopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set directions
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    public void setShooterCoefficients(PIDFCoefficients coefficients){
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

    }

    public void setShooterVelocity(double velocity){
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);
    }

    public double getShooterVelocity() {
       return shooterLeft.getVelocity();
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
