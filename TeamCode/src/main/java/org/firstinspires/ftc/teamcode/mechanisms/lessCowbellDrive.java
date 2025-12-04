package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lessCowbellDrive {
    private DcMotor DriveFL;
    private DcMotor DriveBL;
    private DcMotor DriveFR;
    private DcMotor DriveBR;


    public void init(HardwareMap hwMap) {
        // create drive motors
        // (NOTE!!!!: IF YOU CHANGE THE MOTOR NAMES IN THE ROBOT CONFIG, CHANGE THE deviceName STRING!!!
        DriveFL = hwMap.get(DcMotor.class, "front_left_drive");
        DriveBL = hwMap.get(DcMotor.class, "back_left_drive");
        DriveFR = hwMap.get(DcMotor.class, "front_right_drive");
        DriveBR = hwMap.get(DcMotor.class, "back_right_drive");

        // directions
        DriveFL.setDirection(DcMotor.Direction.FORWARD);
        DriveBL.setDirection(DcMotor.Direction.FORWARD);
        DriveFR.setDirection(DcMotor.Direction.REVERSE);
        DriveBR.setDirection(DcMotor.Direction.REVERSE);

       // mode set thingy
        DriveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    //set functions for motor speed, values -1.0 to 1.0
    public void setDriveFL(double speed) {
        DriveFL.setPower(speed);
    }
    public void setDriveBL(double speed) {
        DriveBL.setPower(speed);
    }
    public void setDriveFR(double speed) {
        DriveFR.setPower(speed);
    }
    public void setDriveBR(double speed) {
        DriveBR.setPower(speed);
    }

}
