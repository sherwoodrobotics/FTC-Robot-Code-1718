package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;
import org.firstinspires.ftc.teamcode.marvframework.MarvTeleOp;

/**
 * Created by Andy on 11/4/17.
 */

@TeleOp(name = "ArmBotTeleOp", group = "ArmBot")
public class ArmBotTeleOp extends MarvTeleOp {
    private static final float 
        
        //Drive motor speeds
        drive_speed = 1.0f,
        slow_drive_speed = 0.25f,
        slow_turn_speed = 0.4f,
    
        //Arm mech motor speeds
        shoulder_arm_speed = 1.0f, //was all .65
        elbow_arm_speed = 1.0f,
        wrist_arm_speed = 1.0f,

        //Servo speed
        servo_hand_speed = 1.0f,
        servo_ls_speed = 1.0f,
        
        //Servo bounds for lightsaber
        servo_yaw_min = 0.40f, 
        servo_yaw_max = 0.85f,
        servo_pitch_min = 0.03f,
        servo_pitch_max = 0.66f,
        
        //Servo bounds for hands
        left_hand_servo_min = 0.05f, 
        right_hand_servo_min = 0.2f,
        left_hand_servo_max = 0.85f,
        right_hand_servo_max = 0.99f,
        
        //Arm drive gear reductions
        shoulder_dr = 27f,
        elbow_dr = 18f,
        wrist_dr = 9f;

    private static long last_time = 0l, current_time = 0l;

    private static final int
        MOTOR_TICKS = 1120,
        ticks_to_move = 500,
        deg_to_move = 25;
    
    private static final float trigger_threshold = 0.15f;
    
    private ArmBotHardware hardware = ArmBotHardware.getHardware();
    private float 
        leftHandPosition = 0f, rightHandPosition = 0f,
        lightsaberPitch = 0f, lightsaberYaw = 0f;    

    @Override
    public MarvHardwareMap getHardwareMap() {
        return hardware;
    }

    public void initTeleOp() { }
    
    @Override
    public void start() {
        leftHandPosition = 0.26f;
        rightHandPosition = 0.72f;
        lightsaberPitch = 0.04f;
        lightsaberYaw = 0.6f;
        hardware.leftHandServo.setPosition(leftHandPosition);
        hardware.rightHandServo.setPosition(rightHandPosition);
        hardware.leftHandServoB.setPosition(1.0f - leftHandPosition);
        hardware.rightHandServoB.setPosition(1.0f - rightHandPosition);
        hardware.lightsaberPitchServo.setPosition(lightsaberPitch);
        hardware.lightsaberYawServo.setPosition(lightsaberYaw);
        
        last_time = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        //Speed control for Servos
        current_time = System.currentTimeMillis();
        float speed = (current_time - last_time) / 1000f;

        //Wheels - both DPad and joystick tank
        if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
            //Joystick drive
            hardware.leftWheelFront.setPower(-gamepad1.left_stick_y * drive_speed);
            hardware.leftWheelBack.setPower(-gamepad1.left_stick_y * drive_speed);
            hardware.rightWheelFront.setPower(-gamepad1.right_stick_y * drive_speed);
            hardware.rightWheelBack.setPower(-gamepad1.right_stick_y * drive_speed);
        } else {
            float leftWheel = 0, rightWheel = 0;
            if (gamepad2.dpad_left || gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.dpad_right) {
                if ((gamepad1.dpad_left && gamepad1.dpad_right) 
                || (gamepad2.dpad_left && gamepad2.dpad_right)
                || (gamepad1.dpad_left && gamepad2.dpad_right)
                || (gamepad2.dpad_left && gamepad1.dpad_right)) {
                    leftWheel = 0;
                    rightWheel = 0;
                } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    leftWheel = -slow_turn_speed;
                    rightWheel = slow_turn_speed;
                } else {
                    leftWheel = slow_turn_speed;
                    rightWheel = -slow_turn_speed;
                }
            } else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad2.dpad_up || gamepad2.dpad_down) {
                if ((gamepad1.dpad_up && gamepad1.dpad_down)
                || (gamepad1.dpad_up && gamepad2.dpad_down)
                || (gamepad2.dpad_up && gamepad1.dpad_down)
                || (gamepad2.dpad_up && gamepad2.dpad_down)) {
                    leftWheel = 0;
                    rightWheel = 0;
                } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    leftWheel = slow_drive_speed;
                    rightWheel = slow_drive_speed;
                } else {
                    leftWheel = -slow_drive_speed;
                    rightWheel = -slow_drive_speed;
                }
            } else {
                leftWheel = 0;
                rightWheel = 0;
            }
            hardware.leftWheelFront.setPower(leftWheel);
            hardware.leftWheelBack.setPower(leftWheel);
            hardware.rightWheelFront.setPower(rightWheel);
            hardware.rightWheelBack.setPower(rightWheel);
        }

        //Lightsaber - abxy
        if (gamepad1.b) {
            //Come out
            lightsaberYaw = clamp(servo_yaw_min, servo_yaw_max, lightsaberYaw + servo_ls_speed * speed);
            hardware.lightsaberYawServo.setPosition(lightsaberYaw);
        } else if (gamepad1.x) {
            lightsaberYaw = clamp(servo_yaw_min, servo_yaw_max, lightsaberYaw - servo_ls_speed * speed);
            hardware.lightsaberYawServo.setPosition(lightsaberYaw);
        }
        if (gamepad1.y) {
            lightsaberPitch = clamp(servo_pitch_min, servo_pitch_max, lightsaberPitch + servo_ls_speed * speed);
            hardware.lightsaberPitchServo.setPosition(lightsaberPitch);
        } else if (gamepad1.a) {
            lightsaberPitch = clamp(servo_pitch_min, servo_pitch_max, lightsaberPitch - servo_ls_speed * speed);
            hardware.lightsaberPitchServo.setPosition(lightsaberPitch);
        }

        //Shoulder + elbow - right joystick + guide
        if (gamepad2.left_stick_y != 0) {
            float motorSpeed = gamepad2.left_stick_y * shoulder_arm_speed;
            hardware.shoulderMotor1.setPower(motorSpeed);
            hardware.shoulderMotor2.setPower(motorSpeed);
            hardware.wristMotor.setPower(motorSpeed * -0.125 * shoulder_dr / wrist_dr);
        } else {
            hardware.shoulderMotor1.setPower(0);
            hardware.shoulderMotor2.setPower(0);
        }
        if (gamepad2.right_stick_y != 0) {
            float motorSpeed = gamepad2.right_stick_y * elbow_arm_speed;
            hardware.elbowMotor.setPower(motorSpeed);
            hardware.wristMotor.setPower(motorSpeed * -0.25 * elbow_dr / wrist_dr);
        } else {
            hardware.elbowMotor.setPower(0);
        }
        if (gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0) {
            hardware.wristMotor.setPower(0);
        }
        
        //Wrist motor - ay
        if (gamepad2.a) {
            hardware.wristMotor.setPower(wrist_arm_speed);
        } else if (gamepad2.y) {
            hardware.wristMotor.setPower(-wrist_arm_speed);
        } else {
            hardware.wristMotor.setPower(0);
        }
        
        //Left servo - trigger closes (c), bumper opens (cc)
        if (gamepad2.left_bumper) {
            leftHandPosition = clamp(left_hand_servo_min, left_hand_servo_max, leftHandPosition + servo_hand_speed * speed);
            hardware.leftHandServo.setPosition(leftHandPosition);
            hardware.leftHandServoB.setPosition(1.0-leftHandPosition);
        } else if (gamepad2.left_trigger > trigger_threshold) {
            leftHandPosition = clamp(left_hand_servo_min, left_hand_servo_max, leftHandPosition - servo_hand_speed * gamepad2.left_trigger * speed);
            hardware.leftHandServo.setPosition(leftHandPosition);
            hardware.leftHandServoB.setPosition(1.0-leftHandPosition);
        }

        //Right servo - trigger closes (cc), bumper opens (c)
        if (gamepad2.right_bumper) {
            rightHandPosition = clamp(right_hand_servo_min, right_hand_servo_max, rightHandPosition - servo_hand_speed * speed);
            hardware.rightHandServo.setPosition(rightHandPosition);
            hardware.rightHandServoB.setPosition(1.0f - rightHandPosition);
        } else if (gamepad2.right_trigger > trigger_threshold) {
            rightHandPosition = clamp(right_hand_servo_min, right_hand_servo_max, rightHandPosition + servo_hand_speed * gamepad2.right_trigger * speed);
            hardware.rightHandServo.setPosition(rightHandPosition);
            hardware.rightHandServoB.setPosition(1.0f - rightHandPosition);
        } 
        
        //Display Servo and sensor values
       telemetry.addLine()
            .addData("hand_left", leftHandPosition)
            .addData("hand_right", rightHandPosition);
        telemetry.addLine()
            .addData("ls_pitch", lightsaberPitch)
            .addData("ls_yaw", lightsaberYaw);
        //telemetry.addLine()
        //    .addData("red", hardware.colorSensor.red())
        //    .addData("green", hardware.colorSensor.green())
        //    .addData("blue", hardware.colorSensor.blue());
        
        telemetry.update(); 
        last_time = current_time;
    }

    public static float clamp(float lowerBound, float upperBound, float value) {
        return (float) Math.max(lowerBound, Math.min(upperBound, value));
    }

    @Override
    public void stop() {
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
