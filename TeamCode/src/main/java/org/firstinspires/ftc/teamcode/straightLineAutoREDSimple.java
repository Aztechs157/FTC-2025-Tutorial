/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.lessCowbellDrive;
import org.firstinspires.ftc.teamcode.mechanisms.lessCowbellIntake;
import org.firstinspires.ftc.teamcode.mechanisms.lessCowbellShooterPIDF;
import org.firstinspires.ftc.teamcode.mechanisms.lessCowbellSpindexer;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="3 Ball Auto - RED", group="Robot")
public class straightLineAutoREDSimple extends LinearOpMode {
     lessCowbellDrive drive = new lessCowbellDrive();
     lessCowbellShooterPIDF shooter = new lessCowbellShooterPIDF();
     lessCowbellSpindexer spindexer = new lessCowbellSpindexer();

     lessCowbellIntake intake = new lessCowbellIntake();

    /* Declare OpMode members. */

    private ElapsedTime     runtime = new ElapsedTime();
    public static double shunitRatio = 11/16.25;



    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        spindexer.init(hardwareMap);
        intake.init(hardwareMap);
        drive.setDriveBLCurrentPosition();
        drive.setDriveBRCurrentPosition();
        drive.setDriveFRCurrentPosition();
        drive.setDriveFLCurrentPosition();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d , %7d :%7d",
                (int)drive.getDriveBLCurrentPosition(),
                (int)drive.getDriveBRCurrentPosition(),
                (int)drive.getDriveFRCurrentPosition(),
                (int)drive.getDriveFLCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//         encoderTranslate(drive.DRIVE_SPEED,  50,  -50, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(drive.DRIVE_SPEED, -15, -15, 5.0);
        shoot(1500, 3);
//        encoderDrive(drive.DRIVE_SPEED,  -40,  -40, 5.0); //
        encoderDrive(drive.DRIVE_SPEED,  35,  -35, 5.0); //
        intake(1);
        encoderDrive(drive.DRIVE_SPEED*0.5,  30,  30, 5.0);
//        encoderDrive(drive.DRIVE_SPEED,  -45,  -45, 5.0);
//        encoderDrive(drive.DRIVE_SPEED,  15,  -15, 5.0); //
//        intake(0);
//        encoderDrive(drive.DRIVE_SPEED,  45,  45, 5.0); //
//        shoot(1500, 3);
//        encoderTranslate(drive.DRIVE_SPEED, -10, 10, 5.0);

//        encoderDrive(drive.TURN_SPEED,   24, -24, 4.0);  // S2: Turn Right 24 Inches with 4 Sec timeout
//        encoderDrive(drive.DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to shoot
     *  1) Start Shooter flywheel
     *  2) Start Hopper
     *  3) Rotate Spindexer to force balls up to shooter
     */

    public void shoot(double speed,
                      int num_balls) {
        for(int i = 0; i<= num_balls; i++) {
            shooter.setShooterVelocity(speed);
            sleep(1000);
            shooter.setHopperSpeed(-1);
            spindexer.setSpindexerSpeed(1);
            sleep(500);
            spindexer.setSpindexerSpeed(0);
            shooter.setHopperSpeed(0);
        }
        shooter.setShooterSpeed(0);
    }
    public void intake(double speed) {

            intake.setIntakeSpeed(speed);
            spindexer.setSpindexerSpeed(speed);
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = drive.setGetDriveBLCurrentPosition() + (int)(leftInches * shunitRatio * drive.COUNTS_PER_INCH);
            newRightTarget = drive.setGetDriveBRCurrentPosition() + (int)(rightInches * shunitRatio * drive.COUNTS_PER_INCH);
            drive.setBLTargetPosition(newLeftTarget);
            drive.setBRTargetPosition(newRightTarget);
            drive.setFLTargetPosition(newLeftTarget);
            drive.setFRTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.setDriveFRModeToPosition();
            drive.setDriveFLModeToPosition();
            drive.setDriveBRModeToPosition();
            drive.setDriveBLModeToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            drive.setDriveFL(Math.abs(speed));
            drive.setDriveFR(Math.abs(speed));
            drive.setDriveBL(Math.abs(speed));
            drive.setDriveBR(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (drive.isFLBusy() && drive.isFRBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        (int)drive.setGetDriveBLCurrentPosition(), (int)drive.setGetDriveBRCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            drive.setDriveBL(0);
            drive.setDriveBR(0);
            drive.setDriveFL(0);
            drive.setDriveFR(0);

            // Turn off RUN_TO_POSITION
            drive.setDriveFLModeToEncoder();
            drive.setDriveFRModeToEncoder();
            drive.setDriveBLModeToEncoder();
            drive.setDriveBRModeToEncoder();

            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderTranslate(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = drive.setGetDriveBLCurrentPosition() - (int)(leftInches * shunitRatio * drive.COUNTS_PER_INCH);
            newRightTarget = drive.setGetDriveBRCurrentPosition() + (int)(rightInches * shunitRatio * drive.COUNTS_PER_INCH);
            drive.setBLTargetPosition(newLeftTarget);
            drive.setBRTargetPosition(newRightTarget);
            drive.setFLTargetPosition(newLeftTarget);
            drive.setFRTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.setDriveFRModeToPosition();
            drive.setDriveFLModeToPosition();
            drive.setDriveBRModeToPosition();
            drive.setDriveBLModeToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            drive.setDriveFL(Math.abs(speed));
            drive.setDriveFR(-Math.abs(speed));
            drive.setDriveBL(-Math.abs(speed));
            drive.setDriveBR(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (drive.isFLBusy() && drive.isFRBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        (int)drive.setGetDriveBLCurrentPosition(), (int)drive.setGetDriveBRCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            drive.setDriveBL(0);
            drive.setDriveBR(0);
            drive.setDriveFL(0);
            drive.setDriveFR(0);

            // Turn off RUN_TO_POSITION
            drive.setDriveFLModeToEncoder();
            drive.setDriveFRModeToEncoder();
            drive.setDriveBLModeToEncoder();
            drive.setDriveBRModeToEncoder();

            sleep(250);   // optional pause after each move.
        }
    }
}
