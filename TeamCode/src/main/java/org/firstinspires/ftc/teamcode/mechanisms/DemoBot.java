package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DemoBot {
    private DigitalChannel touchSensor; // touch sensor on da front bar doohickey
    private DcMotor leftDriveMotor; // left drive motor
    private DcMotor rightDriveMotor; // right drive motor


    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        leftDriveMotor = hwMap.get(DcMotor.class, "left_motor");
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDriveMotor = hwMap.get(DcMotor.class, "right_motor");
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean getTouchSensorState() {
        return touchSensor.getState();
    }

    public void setLeftMotorSpeed(double speed){
        // -1.0 to 1.0
        leftDriveMotor.setPower(speed);
    }

    public void setRightMotorSpeed(double speed){
        // -1.0 to 1.0
        rightDriveMotor.setPower(speed);
    }
}
