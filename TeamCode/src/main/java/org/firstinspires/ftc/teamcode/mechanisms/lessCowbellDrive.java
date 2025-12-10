package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lessCowbellDrive {
    private DcMotor DriveFL;
    private DcMotor DriveBL;
    private DcMotor DriveFR;
    private DcMotor DriveBR;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: REV HEX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 12 ;     // 12:1 Gear Ratio
    public static final double     WHEEL_DIAMETER_INCHES   = 3.543 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.6;
    public static final double     TURN_SPEED              = 0.6;

    public static double positionBL;
    public static double positionBR;
    public static double positionFL;
    public static double positionFR;

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
        DriveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void setDriveFRModeToPosition(){
        DriveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setDriveFLModeToPosition(){
        DriveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setDriveBRModeToPosition(){
        DriveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setDriveBLModeToPosition(){
        DriveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDriveFRModeToEncoder(){
        DriveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDriveFLModeToEncoder(){
        DriveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDriveBRModeToEncoder(){
        DriveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDriveBLModeToEncoder(){
        DriveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isFLBusy(){
        return DriveFL.isBusy();
    }
    public boolean isFRBusy(){
        return DriveFR.isBusy();
    }
    public boolean isBLBusy(){
        return DriveBL.isBusy();
    }
    public boolean isBRBusy(){
        return DriveBR.isBusy();
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

    public void setDriveBLCurrentPosition(){
        positionBL = DriveBL.getCurrentPosition();
    }
    public double getDriveBLCurrentPosition(){
        return positionBL;
    }
    public int setGetDriveBLCurrentPosition(){
        setDriveBLCurrentPosition();
        return (int)getDriveBLCurrentPosition();
    }
    public void setBLTargetPosition(int position){
        DriveBL.setTargetPosition(position);
    }
    public void setDriveBRCurrentPosition(){
        positionBR = DriveBR.getCurrentPosition();
    }
    public double getDriveBRCurrentPosition(){
        return positionBR;
    }
    public int setGetDriveBRCurrentPosition(){
        setDriveBRCurrentPosition();
        return (int)getDriveBRCurrentPosition();
    }
    public void setBRTargetPosition(int position){
        DriveBR.setTargetPosition(position);
    }
    public void setDriveFLCurrentPosition(){
        positionFL = DriveFL.getCurrentPosition();
    }
    public double getDriveFLCurrentPosition(){
        return positionFL;
    }
    public void setFLTargetPosition(int position){
        DriveFL.setTargetPosition(position);
    }
    public void setDriveFRCurrentPosition(){
        positionFR = DriveFR.getCurrentPosition();
    }
    public double getDriveFRCurrentPosition(){
        return positionFR;
    }
    public void setFRTargetPosition(int position){
        DriveFR.setTargetPosition(position);
    }
}
