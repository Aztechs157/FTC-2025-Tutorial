package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.lessCowbellWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class webCamTest extends OpMode {
    lessCowbellWebcam webcam = new lessCowbellWebcam();
    @Override
    public void init() {
        webcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        webcam.update();
        AprilTagDetection id20 = webcam.getTagbySpecificId(20);
        webcam.displayDetectionTelemetry(id20);

    }
}
