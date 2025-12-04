package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class servoTest extends OpMode {
    private CRServo spindexer;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    @Override
    public void init() {

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        spindexer = hardwareMap.get(CRServo.class, "spindexer");

    }

    @Override
    public void loop() {

    }
}
