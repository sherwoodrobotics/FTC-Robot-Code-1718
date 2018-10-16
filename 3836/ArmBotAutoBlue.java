package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.marvframework.MarvAutonomous;
import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;
import org.firstinspires.ftc.teamcode.marvframework.utils.VuforiaIdentifier;

@Autonomous(name = "ArmBotAutoBlue", group = "ArmBot")
public class ArmBotAutoBlue extends MarvAutonomous {
    
    private ArmBotHardware hardware = ArmBotHardware.getHardware();

    private static final int MOTOR_TICKS = 1120;

    public static final float
        //Wheel ticks & encoder values
        WHEEL_DIAMETER_FRONT = 4.0f,
        WHEEL_DIAMETER_BACK = 4.25f,
        TICKS_PER_INCH_FRONT = MOTOR_TICKS / WHEEL_DIAMETER_FRONT / 3.1415f,
        TICKS_PER_INCH_BACK = MOTOR_TICKS / WHEEL_DIAMETER_BACK / 3.1415f,
        DRIVE_SPEED = 0.7f;

    @Override
    public MarvHardwareMap getHardwareMap() {
        return hardware;
    }

    @Override
    public void initAutonomous() {
        
        hardware.initAutonomous(hardwareMap);
        
        hardware.leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }

    @Override
    public void runAutonomous() {
        //Deploy lightsaber
        hardware.lightsaberYawServo.setPosition(0.65f);
        Wait(0.5f);
        hardware.lightsaberPitchServo.setPosition(0.66f);
        Wait(2f);
                
        //Read color sensor, hit jewel
        if (hardware.colorSensor.blue() > 1.75f * hardware.colorSensor.red()) {
            //Blue on right
            hardware.lightsaberYawServo.setPosition(0.5f);
            Wait(1f);
        } else if (hardware.colorSensor.red() > 1.75f * hardware.colorSensor.blue()){
            //Red on right
            hardware.lightsaberYawServo.setPosition(0.8f);
            Wait(1f);
        } //Else not enough info, no action
        
        //Retract lightsaber
        hardware.lightsaberPitchServo.setPosition(0.3f);
        Wait(0.5f);
        hardware.lightsaberYawServo.setPosition(0.5f);
        Wait(0.5f);
        hardware.lightsaberPitchServo.setPosition(0.04f);
        Wait(1f);
        
        //Drive into triangle
        drive(DRIVE_SPEED, -32.5f);
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
