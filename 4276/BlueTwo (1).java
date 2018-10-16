package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.marvframework.MarvAutonomous;
import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;

import org.firstinspires.ftc.teamcode.marvframework.MarvTeleOp;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.marvframework.utils.VuforiaIdentifier;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Benjamin Rome on 1/15/18 Edited on 2/3/18
 *
 * Autonomous that uses a color sensor to knock jewel off platform; then drives, turns,
 * and places block based on the pictograph
 */

@Autonomous(name = "BlueTwo", group = "Blue")
public class BlueTwo extends MarvAutonomous {
    
    private BlueBotHardware hardware = BlueBotHardware.getHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440 ; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.125 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED  = 0.5;
    
    //Our VurofiaIdentifier
    VuforiaIdentifier vuforia;

    public static final float 
        turn_update_time = 0.1f, //Seconds, how long between each gyro update
        turn_radius = 90f, //Degrees, how much to turn each time
        drive_speed = 0.75f, //How strong to run the motors while driving forward
        turn_speed = 0.55f, //How strong to run the motors while turning (max)
        error_threshold = 1.75f; //Degrees, max error allowed before turn considered complete
        
    @Override
    public MarvHardwareMap getHardwareMap() {
        return hardware;
    }

    @Override
    public void initAutonomous() {
        
        vuforia = new VuforiaIdentifier(hardwareMap, true, "AUkLfjn/////AAAAGUFtPQ6KGksLs8oQ2l5Pe1g/KWU/uFlnUVPpiE2CB16FMufeFG5qtSLqUbJDyutkf7mRFjv6/ZaaN9MRPuO2T5QU63QmEc7lokejbDmwdWU1KwzcdjdWfYmdrv04OFY2fSE0Afud6gBfPoWTLQQf1NHC8SDc+TAo1k5fGeKbzWYeAyqn1lMWL/UXelsa5pc+uZSsCHB41GiuikQ49Z8aJ0IVyT6u+G3GgpiXWtD3O7zwtPTD6RcRrOnanw7LlE04mUb6/2SKxP3MuOcTtdSn7uAradqrg5f8PAT8E9UaPnsjlZWXSxrwCotiJykpVQ5CYwdcoiEO3x+FJWDlDk5GCLWhpK4jE/gJ/X+2PVK6fPti");
        vuforia.run();
    }
    
    // What is going to be run in autonomous
    @Override
    public void runAutonomous() {
        
        initalGrab(); // Grab the block at the start and raise the lift
        
        lightsaberActivate(); // Color Sensor
        
        Wait(2.0f);
        
        //This will either return LEFT, CENTER, RIGHT, or UNKNOWN
        telemetry.addLine().addData("detected mark", vuforia.isMarkerFound().toString());
        
        telemetry.update();
       
        switch (vuforia.isMarkerFound().toString()) {
            case "LEFT":
                leftBlueTwo(); // All of the code to place the block in the left column
                break;
            case "RIGHT":
                rightBlueTwo(); // All of the code to place the block in the right column
                break;
            case "CENTER":
                centerBlueTwo(); // All of the code to place the block in the center column
                break;
            default:
                leftBlueTwo();
                break;
            }
    }
    
    public void vuforia() {
        Wait(2.0f);
        
        telemetry.clearAll();
        
        //This will either return LEFT, CENTER, RIGHT, or UNKNOWN
        telemetry.addLine().addData("detected mark", vuforia.isMarkerFound().toString());
        
        telemetry.update();
    }
    
    // This is how the robot drives forward based on motor power, and inches traveled.
    public void encoderDriveBlue(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = hardware.leftWheel.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = hardware.rightWheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            hardware.leftWheel.setTargetPosition(newLeftTarget);
            hardware.rightWheel.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            hardware.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            hardware.leftWheel.setPower(Math.abs(speed));
            hardware.rightWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (hardware.leftWheel.isBusy() && hardware.rightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            hardware.leftWheel.getCurrentPosition(),
                                            hardware.rightWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            hardware.leftWheel.setPower(0);
            hardware.rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            hardware.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        
        }
    }
    
    public void centerBlueTwo() {
        
        encoderDriveBlue(0.8, -20.0, -20.0, 10.0);
        
        Wait(1.0f);
        
        turn(-90f);
        
        encoderDriveBlue(0.7, -20.0, -22.0, 15.0);
        
        encoderDriveBlue(0.7, -2.0, -2.0, 15.0);
        
        turn(-35f);
        
        encoderDriveBlue(0.4, -4.0, -4.0, 2.0);
        
        hardware.topLeftArm.setPosition(0.34f);
        hardware.topRightArm.setPosition(0.65f);
        Wait(1.5f);
        
        encoderDriveBlue(0.7, -4.0, -4.0, 2.0);
        
        hardware.upDown.setPower(0.5f);
        Wait(0.25f);
        hardware.upDown.setPower(0.0f);
        Wait(0.5f);
        
        hardware.topLeftArm.setPosition(0.6f);
        hardware.topRightArm.setPosition(0.3f);
        Wait(1.5f);
        
        encoderDriveBlue(0.28, 11.0, 11.0, 3.0);
        
        encoderDriveBlue(0.4, -4.0, -4.0, 3.0);
    }
    
     public void rightBlueTwo() {
        
       encoderDriveBlue(0.8, -20.0, -20.0, 10.0);
        
        Wait(1.0f);
        
        turn(-91f);
        
        encoderDriveBlue(0.7, -22.0, -24.0, 15.0);
        
        encoderDriveBlue(0.7, -2.0, -2.0, 15.0);
        
        turn(-45f);
        
        encoderDriveBlue(0.4, -4.0, -4.0, 2.0);
        
        hardware.topLeftArm.setPosition(0.34f);
        hardware.topRightArm.setPosition(0.65f);
        Wait(1.5f);
        
        encoderDriveBlue(0.7, -4.0, -4.0, 2.0);
        
        hardware.upDown.setPower(0.5f);
        Wait(0.25f);
        hardware.upDown.setPower(0.0f);
        Wait(0.5f);
        
        hardware.topLeftArm.setPosition(0.6f);
        hardware.topRightArm.setPosition(0.3f);
        Wait(1.5f);
        
        encoderDriveBlue(0.28, 11.0, 11.0, 3.0);
        
        encoderDriveBlue(0.4, -2.0, -2.0, 1.0);
    }
    
    public void leftBlueTwo() {
        
        encoderDriveBlue(0.8, -20.0, -20.0, 10.0);
        
        Wait(1.0f);
        
        turn(-91f);
        
        encoderDriveBlue(0.7, -12.0, -14.0, 15.0);
        
        encoderDriveBlue(0.7, -2.0, -2.0, 15.0);
        
        turn(-33f);
        
        encoderDriveBlue(0.4, -4.0, -4.0, 2.0);
        
        hardware.topLeftArm.setPosition(0.34f);
        hardware.topRightArm.setPosition(0.65f);
        Wait(1.5f);
        
        encoderDriveBlue(0.7, -4.0, -4.0, 2.0);
        
        hardware.upDown.setPower(0.5f);
        Wait(0.25f);
        hardware.upDown.setPower(0.0f);
        Wait(0.5f);
        
        hardware.topLeftArm.setPosition(0.6f);
        hardware.topRightArm.setPosition(0.3f);
        Wait(1.5f);
        
        encoderDriveBlue(0.28, 11.0, 11.0, 3.0);
        
        encoderDriveBlue(0.4, -4.0, -4.0, 3.0);
    }
    
    /* The beginning of autonomous, bottom two arms 
    grab the block and the lift lifts */
    
    public void initalGrab() {
        hardware.leftArm.setPosition(0.7f);
        hardware.rightArm.setPosition(0.3f);
        Wait(1.0f);
        
        hardware.topLeftArm.setPosition(0.6f);
        hardware.topRightArm.setPosition(0.4f);
        Wait(1.5f);
        
        hardware.upDown.setPower(-0.5f);
        Wait(1.0f);
        hardware.upDown.setPower(0.0f);
    }
    
    public void lightsaberActivate() {
                
        //Deploy lightsaber
        hardware.lightsaberYawServo.setPosition(0.24f); // Yaw == left/right
        Wait(0.5f);
        hardware.lightsaberPitchServo.setPosition(0.46f); // Pitch == up/down
        Wait(2f);
                
        // Read color sensor, hit jewel
        if (hardware.colorSensor.blue() > 1.5f * hardware.colorSensor.red()) {
                    
            // Blue on right
            hardware.lightsaberYawServo.setPosition(0.0f);
            Wait(1f);
        } else if (hardware.colorSensor.red() > 1.5f * hardware.colorSensor.blue()){
                    
            // Red on right
            hardware.lightsaberYawServo.setPosition(0.8f);
            Wait(1f);
                
        } // Else not enough info, no action
        
        // Retract lightsaber
        hardware.lightsaberPitchServo.setPosition(0.6f);
        Wait(0.5f);
        hardware.lightsaberYawServo.setPosition(0.24f);
        Wait(0.5f);        
        hardware.lightsaberPitchServo.setPosition(1.4f);
        Wait(0.5f);
        hardware.lightsaberYawServo.setPosition(0.5f);
        Wait(0.5f);
        }
    
    public void turn(float degrees) {
        
        //Prepare for turn
        boolean turnComplete = false;
        float error;
        float targetAngle = hardware.gyroSensor.getHeading() + degrees;
        Wait(turn_update_time);
        
        //Perform turn
        while (!turnComplete && !shouldStop()) {
            //Calculate error and check threshold
            hardware.gyroSensor.update(); //Calculates the rotation angles of the gyro
            error = targetAngle - hardware.gyroSensor.getHeading();
            if (Math.abs(error) <= error_threshold) turnComplete = true;
            
            else {
                //As error gest smaller, lower speed
                if (error < -30f) {
                    hardware.leftWheel.setPower(turn_speed);
                    hardware.rightWheel.setPower(-turn_speed);
                } else if (error > 30f) {
                    hardware.leftWheel.setPower(-turn_speed);
                    hardware.rightWheel.setPower(turn_speed);    
                } else if (error < -20f) {
                    hardware.leftWheel.setPower(turn_speed / 2f);
                    hardware.rightWheel.setPower(-turn_speed / 2f);
                } else if (error > 20f) {
                    hardware.leftWheel.setPower(-turn_speed / 2f);
                    hardware.rightWheel.setPower(turn_speed / 2f);
                } else if (error < -10f) {
                    hardware.leftWheel.setPower(turn_speed / 3f);
                    hardware.rightWheel.setPower(-turn_speed / 3f);
                } else if (error > 10f) {
                    hardware.leftWheel.setPower(-turn_speed / 3f);
                    hardware.rightWheel.setPower(turn_speed / 3f);
                } 
                
                Wait(turn_update_time);
            }
        }
        //Finished turn
        hardware.leftWheel.setPower(0);
        hardware.rightWheel.setPower(0);
    }
    
    @Override
    public void stopAutonomous() {
        hardware.leftWheel.setPower(0);
        hardware.rightWheel.setPower(0);
    }
}



