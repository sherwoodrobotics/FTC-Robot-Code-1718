package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.marvframework.MarvAutonomous;
import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;
import org.firstinspires.ftc.teamcode.marvframework.utils.VuforiaIdentifier;

@Autonomous(name = "ArmBotBlue2", group = "ArmBot")
public class ArmBotBlue2 extends MarvAutonomous {

private static final int MOTOR_TICKS = 1120;

    public static final float
        //Wheel ticks & encoder values
        WHEEL_DIAMETER_FRONT = 4.0f,
        WHEEL_DIAMETER_BACK = 4.25f,
        TICKS_PER_INCH_FRONT = MOTOR_TICKS / WHEEL_DIAMETER_FRONT / 3.1415f,
        TICKS_PER_INCH_BACK = MOTOR_TICKS / WHEEL_DIAMETER_BACK / 3.1415f,
        DRIVE_SPEED = 0.6f,
        TURN_SPEED = 0.7f,
        
        //Arm drive gear reductions
        shoulder_dr = 27f,
        elbow_dr = 18f,
        wrist_dr = 9f,
        
        //Arm mech motor speeds
        shoulder_arm_speed = 1.0f,
        elbow_arm_speed = 1.0f,
        wrist_arm_speed = 1.0f,
        
        //Gyro turning
        turn_update_time = 0.075f,
        error_threshold = 1.75f;
    
    private float error1 = 0f, error2 = 0f;
    
    private ArmBotHardware hardware = ArmBotHardware.getHardware();
    VuforiaIdentifier vuforia = null;

    @Override
    public MarvHardwareMap getHardwareMap() {
        return hardware;
    }

    @Override
    public void initAutonomous() {
        
        hardware.initAutonomous(hardwareMap);
        
        hardware.shoulderMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.shoulderMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vuforia = new VuforiaIdentifier(hardwareMap, true, "AUkLfjn/////AAAAGUFtPQ6KGksLs8oQ2l5Pe1g/KWU/uFlnUVPpiE2CB16FMufeFG5qtSLqUbJDyutkf7mRFjv6/ZaaN9MRPuO2T5QU63QmEc7lokejbDmwdWU1KwzcdjdWfYmdrv04OFY2fSE0Afud6gBfPoWTLQQf1NHC8SDc+TAo1k5fGeKbzWYeAyqn1lMWL/UXelsa5pc+uZSsCHB41GiuikQ49Z8aJ0IVyT6u+G3GgpiXWtD3O7zwtPTD6RcRrOnanw7LlE04mUb6/2SKxP3MuOcTtdSn7uAradqrg5f8PAT8E9UaPnsjlZWXSxrwCotiJykpVQ5CYwdcoiEO3x+FJWDlDk5GCLWhpK4jE/gJ/X+2PVK6fPti");

        hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.shoulderMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.shoulderMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        vuforia.run();
    }

    @Override
    public void runAutonomous() {
        hardware.lightsaberPitchServo.setPosition(0.04f);
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                //Deploy lightsaber
                hardware.lightsaberYawServo.setPosition(0.65f);
                Wait(0.5f);
                hardware.lightsaberPitchServo.setPosition(0.66f);
                Wait(3.25f);
                
                //Read color sensor, hit jewel
                if (hardware.colorSensor.blue() > 1.75f * hardware.colorSensor.red()) {
                    //Blue on right
                    hardware.lightsaberYawServo.setPosition(0.50f);
                    Wait(1f);
                } else if (hardware.colorSensor.red() > 1.75f * hardware.colorSensor.blue()){
                    //Red on right
                    hardware.lightsaberYawServo.setPosition(0.8f);
                    Wait(1f);
                } //Else not enough info, no action
        
                //Retract lightsaber
                hardware.lightsaberPitchServo.setPosition(0.3f);
                Wait(0.5f);
                hardware.lightsaberYawServo.setPosition(0.65f);
                Wait(0.5f);
                hardware.lightsaberPitchServo.setPosition(0.04f);
                Wait(1f);
            }
        });
        thread.start();
        
        //Pick up block
        moveShoulderMotors(10f);
        moveWristMotors(112f);
        hardware.rightHandServo.setPosition(0.25f);
        hardware.rightHandServoB.setPosition(0.75f);
        hardware.leftHandServo.setPosition(0.75f);
        hardware.leftHandServoB.setPosition(0.25f);
        moveWristMotors(90f);
        moveElbowMotors(5f);
        hardware.rightHandServo.setPosition(0.85f);
        hardware.rightHandServoB.setPosition(0.15f);
        hardware.leftHandServo.setPosition(0.15f);
        hardware.leftHandServoB.setPosition(0.85f);
        Wait(0.5f);
        moveWristMotors(-130f);
        Wait(1.0f);

        //Drive to proper column
        drive(DRIVE_SPEED, -22f);
        turn(TURN_SPEED, -90f);
        String direction = vuforia.isMarkerFound().toString();
        if (direction.equals("UNKNOWN")) direction = "CENTER";
        float distance = 0.0f;
        switch (direction) {
            case "LEFT":
                //Forward 25, over 6
                distance = -15f;
                break;
            case "CENTER":
                //Forward 25, over 13
                distance = -22f;
                break;
            case "RIGHT":
                //Forward 25, over 20
                distance = -29f;
                break;
        }

        //Drive to column
        drive(DRIVE_SPEED, distance);
        turn(TURN_SPEED, -63f);
        drive(DRIVE_SPEED, -16f);

        //Let go of block
        moveElbowMotors(-72f);
        Wait(1f);
        hardware.rightHandServo.setPosition(0.25f);
        hardware.rightHandServoB.setPosition(0.75f);
        hardware.leftHandServo.setPosition(0.75f);
        hardware.leftHandServoB.setPosition(0.25f);
        
        //Back up, raise arm, then drive back forwards
        Wait(1f);
        drive(DRIVE_SPEED, 11.5f);
        Wait(0.5f);
        drive(DRIVE_SPEED, -3f);

        /*
        
        Left
        full open - 0.75f
        closed - 0.16f
        slightly open - 0.4f
        
        Right
        full open - 0.25f
        closed - 0.85f
        sligly open - 0.6f
        
        */
    }

    public void drive(float speed, float distance) {

        if (!shouldStop()) {
            
            //Calculate new targets
            int tick_delta_front = (int) (TICKS_PER_INCH_FRONT * distance),
                tick_delta_back = (int) (TICKS_PER_INCH_BACK * distance),
                newFLTarget = hardware.leftWheelFront.getCurrentPosition() + tick_delta_front,
                newFRTarget = hardware.rightWheelFront.getCurrentPosition() + tick_delta_front,
                newBLTarget = hardware.leftWheelBack.getCurrentPosition() + tick_delta_back,
                newBRTarget = hardware.rightWheelBack.getCurrentPosition() + tick_delta_back;

            //Set new targets
            hardware.leftWheelFront.setTargetPosition(newFLTarget);
            hardware.leftWheelBack.setTargetPosition(newFRTarget);
            hardware.rightWheelFront.setTargetPosition(newBLTarget);
            hardware.rightWheelBack.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start power
            hardware.leftWheelFront.setPower(Math.abs(speed));
            hardware.leftWheelBack.setPower(Math.abs(speed));
            hardware.rightWheelFront.setPower(Math.abs(speed));
            hardware.rightWheelBack.setPower(Math.abs(speed));

            while (!shouldStop() &&
                    (hardware.leftWheelFront.isBusy() &&
                    hardware.leftWheelBack.isBusy() &&
                    hardware.rightWheelFront.isBusy() &&
                    hardware.rightWheelBack.isBusy())) {
                //Stall
            }

            // Stop power
            hardware.leftWheelFront.setPower(0);
            hardware.leftWheelBack.setPower(0);
            hardware.rightWheelFront.setPower(0);
            hardware.rightWheelBack.setPower(0);

            // Turn off RUN_TO_POSITION
            hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Wait(0.1f);   // Seconds
        }
    }
    
    public float gyrosError(float target1, float target2) {
        hardware.gyro1.update();
        hardware.gyro2.update();
        float error1new = target1 - hardware.gyro1.getHeading();
        float error2new = target2 - hardware.gyro2.getHeading();
        
        return error1new;
        /*
        if (Math.abs(error1new - error1) > 5.0f && Math.abs(error2new - error2) <= 5.0f) {
            //Gyro2 assumed dead 
            error2 = 0f;
            error1 = error1new;
            return error1;
        } else if (Math.abs(error1new - error1) <= 5.0f && Math.abs(error2new - error2) > 5.0f) {
            //Gyro1 assumed dead
            error1 = 0f;
            error2 = error2new;
            return error2;
        } else {
            error1 = error1new;
            error2 = error2new;        
            return (error1 + error2) / 2f;
        }
        */
    }

    public void turn(float turn_speed, float degrees) {
        
        if (shouldStop()) {
            return;
        }
        
        hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //Prepare for turn
        hardware.gyro1.update();
        hardware.gyro2.update();
        boolean turnComplete = false;
        float error;
        float targetAngle1 = hardware.gyro1.getHeading() + degrees;
        float targetAngle2 = hardware.gyro2.getHeading() + degrees;
        Wait(turn_update_time);
        
        //Perform turn
        while (!turnComplete && !shouldStop()) {
            //Calculate error and check threshold
            error = gyrosError(targetAngle1, targetAngle2);
            if (Math.abs(error) <= error_threshold) turnComplete = true;
            
            else {
                //As error gets smaller, lower speed
                if (error < -30f) {
                    hardware.leftWheelFront.setPower(turn_speed);
                    hardware.rightWheelFront.setPower(-turn_speed);
                    hardware.leftWheelBack.setPower(turn_speed);
                    hardware.rightWheelBack.setPower(-turn_speed);
                } else if (error > 30f) {
                    hardware.leftWheelFront.setPower(-turn_speed);
                    hardware.rightWheelFront.setPower(turn_speed);
                    hardware.leftWheelBack.setPower(-turn_speed);
                    hardware.rightWheelBack.setPower(turn_speed);
                } else if (error < 0f) {
                    hardware.leftWheelFront.setPower(turn_speed / 2f);
                    hardware.rightWheelFront.setPower(-turn_speed / 2f);
                    hardware.leftWheelBack.setPower(turn_speed / 2f);
                    hardware.rightWheelBack.setPower(-turn_speed / 2f);
                } else if (error > 0f) {
                    hardware.leftWheelFront.setPower(-turn_speed / 2f);
                    hardware.rightWheelFront.setPower(turn_speed / 2f);
                    hardware.leftWheelBack.setPower(-turn_speed / 2f);
                    hardware.rightWheelBack.setPower(turn_speed / 2f);
                } 
                
                Wait(turn_update_time);
            }
        }
        //Finished turn
        hardware.leftWheelFront.setPower(0f);
        hardware.leftWheelBack.setPower(0f);
        hardware.rightWheelFront.setPower(0f);
        hardware.rightWheelBack.setPower(0f);  
        
        //Reset encoders
        Wait(0.2f);
        hardware.leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wait(0.2f);
        hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void moveShoulderMotors(float degrees) {
        
        //Turn shoulder motors - calculate delta
        int ticksToMove = (int) (degrees / 360f * MOTOR_TICKS * shoulder_dr * -1);
        int wristMotorTicks = (int) (ticksToMove * -0.5 * wrist_dr / shoulder_dr);
        float wristMotorSpeed = ticksToMove / (float) wristMotorTicks * shoulder_arm_speed;
        
        telemetry.addLine().addData("ticksToMove", ticksToMove);
        telemetry.addLine().addData("wristMotorTicks", wristMotorTicks);
        telemetry.addLine().addData("motorSpeed", shoulder_arm_speed);
        telemetry.addLine().addData("wristMotorSpeed", wristMotorSpeed);
        telemetry.update();
        
        //Calculate and set new tick targets
        int newShoulder1Target = hardware.shoulderMotor1.getCurrentPosition() + ticksToMove;
        int newShoulder2Target = hardware.shoulderMotor2.getCurrentPosition() + ticksToMove;
        int newWristTarget = hardware.wristMotor.getCurrentPosition() + wristMotorTicks;
        hardware.shoulderMotor1.setTargetPosition(newShoulder1Target);
        hardware.shoulderMotor2.setTargetPosition(newShoulder2Target);
        hardware.wristMotor.setTargetPosition(newWristTarget);
        
        //Run motors
        hardware.shoulderMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.shoulderMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.shoulderMotor1.setPower(Math.abs(shoulder_arm_speed));
        hardware.shoulderMotor2.setPower(Math.abs(shoulder_arm_speed));
        hardware.wristMotor.setPower(Math.abs(wristMotorSpeed));
        
        //Perform movement, then stop
        while (hardware.shoulderMotor1.isBusy() && hardware.shoulderMotor2.isBusy()
                && !shouldStop()) {
            //Stall    
        }
        
        //Stop
        hardware.shoulderMotor1.setPower(0f);
        hardware.shoulderMotor2.setPower(0f);
        hardware.wristMotor.setPower(0f);
        hardware.shoulderMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.shoulderMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void moveElbowMotors(float degrees) {
        //Turn elbow motors - calculate delta
        int ticksToMove = (int) (degrees / 360f * MOTOR_TICKS * elbow_dr * -1);
        int wristMotorTicks = (int) (ticksToMove * -0.5 * wrist_dr / elbow_dr);
        float wristMotorSpeed = ticksToMove / (float)  wristMotorTicks * elbow_arm_speed;
        
        telemetry.addLine().addData("ticksToMove", ticksToMove);
        telemetry.addLine().addData("wristMotorTicks", wristMotorTicks);
        telemetry.addLine().addData("motorSpeed", elbow_arm_speed);
        telemetry.addLine().addData("wristMotorSpeed", wristMotorSpeed);
        telemetry.update();
        
        //Calculate and set new tick targets                
        int newElbowTarget = hardware.elbowMotor.getCurrentPosition() + ticksToMove;
        int newWristTarget = hardware.wristMotor.getCurrentPosition() + wristMotorTicks;
        hardware.elbowMotor.setTargetPosition(newElbowTarget);
        hardware.wristMotor.setTargetPosition(newWristTarget);
                
        //Run motors            
        hardware.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.elbowMotor.setPower(Math.abs(elbow_arm_speed));
        hardware.wristMotor.setPower(Math.abs(wristMotorSpeed));
        
        //Perform movement, then stop
        while (hardware.elbowMotor.isBusy() && !shouldStop()) {
            //Stall
        }
        
        //Stop
        hardware.elbowMotor.setPower(0f);
        hardware.wristMotor.setPower(0f);
        hardware.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void moveWristMotors(float degrees) {
        //Turn shoulder motors - calculate delta
        int ticksToMove = (int) (degrees / 360f * MOTOR_TICKS * wrist_dr * -1.0f);

        //Calculate and set new tick targets
        int newWristTarget = hardware.wristMotor.getCurrentPosition() + ticksToMove;
        hardware.wristMotor.setTargetPosition(newWristTarget);
        
        //Run motors
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.wristMotor.setPower(Math.abs(wrist_arm_speed));
        
        //Perform movement, then stop
        while (hardware.wristMotor.isBusy() && !shouldStop()) {
            //Stall
        }
        
        //Stop
        hardware.wristMotor.setPower(0f);
        hardware.wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stopAutonomous() {
        hardware.leftWheelFront.setPower(0f);
        hardware.leftWheelBack.setPower(0f);
        hardware.rightWheelFront.setPower(0f);
        hardware.rightWheelBack.setPower(0f);
        hardware.shoulderMotor1.setPower(0f);
        hardware.shoulderMotor2.setPower(0f);
        hardware.elbowMotor.setPower(0f);
        hardware.wristMotor.setPower(0f);
    }

}
