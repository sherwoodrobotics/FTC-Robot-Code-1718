package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;
import org.firstinspires.ftc.teamcode.marvframework.utils.RevGyroIMU;

/**
 * Created by Andy on 11/4/17.
 */

public class ArmBotHardware extends MarvHardwareMap {

    public static final int ANDYMARK_MOTOR_TICKS = 1120;

    public static ArmBotHardware botHardware;
    public static ArmBotHardware getHardware() {
        if (botHardware == null) botHardware = new ArmBotHardware();
        return botHardware;
    }
    
    //Drive wheel motors
    public DcMotor leftWheelFront = null;
    public DcMotor rightWheelFront = null;
    public DcMotor leftWheelBack = null;
    public DcMotor rightWheelBack = null;
    
    //Arm mech motors - names based off human arm joints
    public DcMotor shoulderMotor1 = null;
    public DcMotor shoulderMotor2 = null;
    public DcMotor elbowMotor = null;
    public DcMotor wristMotor = null;
    
    //Block grabbing mech
    public Servo leftHandServo = null;
    public Servo rightHandServo = null;
    public Servo leftHandServoB = null;
    public Servo rightHandServoB = null;
    
    //Lightsaber
    public Servo lightsaberPitchServo = null;
    public Servo lightsaberYawServo = null;
    
    //Sensors
    ColorSensor colorSensor = null;
    RevGyroIMU gyro1 = null;
    RevGyroIMU gyro2 = null;
    
    public ArmBotHardware() { }

    @Override
    public void init(HardwareMap hardwareMap) {
        //Dec Motors
        leftWheelFront  = getDevice(hardwareMap, DcMotor.class, "left_wheel");
        rightWheelFront = getDevice(hardwareMap, DcMotor.class, "right_wheel");
        leftWheelBack  = getDevice(hardwareMap, DcMotor.class, "left_wheel_back");
        rightWheelBack = getDevice(hardwareMap, DcMotor.class, "right_wheel_back");
        shoulderMotor1 = getDevice(hardwareMap, DcMotor.class, "shoulder_motor_1");
        shoulderMotor2 = getDevice(hardwareMap, DcMotor.class, "shoulder_motor_2");
        elbowMotor = getDevice(hardwareMap, DcMotor.class, "elbow_motor");
        wristMotor = getDevice(hardwareMap, DcMotor.class, "wrist_motor");
    
        //Motor Directions
        leftWheelFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftWheelBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightWheelBack.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //No Encoders
        leftWheelFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servos
        leftHandServo = getDevice(hardwareMap, Servo.class, "left_hand_servo");
        rightHandServo = getDevice(hardwareMap, Servo.class, "right_hand_servo");
        leftHandServoB = getDevice(hardwareMap, Servo.class, "left_hand");
        rightHandServoB = getDevice(hardwareMap, Servo.class, "right_hand");
        lightsaberPitchServo = getDevice(hardwareMap, Servo.class, "ls_pitch");
        lightsaberYawServo = getDevice(hardwareMap, Servo.class, "ls_yaw");
        
    }
    
    public void initAutonomous(HardwareMap hardwareMap) {
        //Sensors
        colorSensor = getDevice(hardwareMap, ColorSensor.class, "color");
        gyro2 = new RevGyroIMU(hardwareMap, "imu2"); 
        gyro1 = new RevGyroIMU(hardwareMap, "imu1"); 
    
    }
}
