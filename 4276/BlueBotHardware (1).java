package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.marvframework.utils.RevGyroIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;

/**
 * Created by sandy on 11/2/17.
 */

public class BlueBotHardware extends MarvHardwareMap {

    public static BlueBotHardware botHardware;
    public static BlueBotHardware getHardware() {
        if (botHardware == null) botHardware = new BlueBotHardware();
        return botHardware;
    }
    // Motors
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public DcMotor upDown = null;
    public DcMotor relic = null;
    
    // Servos
    public Servo leftArm = null;
    public Servo rightArm = null;
    public Servo topLeftArm = null;
    public Servo topRightArm = null;
    public Servo lightsaberYawServo = null;
    public Servo lightsaberPitchServo = null;
    public Servo relicGrab = null;
    public Servo relicRotate = null;
    
    // Sensors
    public RevGyroIMU gyroSensor = null;
    public ColorSensor colorSensor = null;
    
    public BlueBotHardware() {}

    @Override
    public void init(HardwareMap hardwareMap) {

        gyroSensor = new RevGyroIMU(hardwareMap, "imu");

        leftWheel  = getDevice(hardwareMap, DcMotor.class, "left_wheel");
        rightWheel = getDevice(hardwareMap, DcMotor.class, "right_wheel");
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        upDown = getDevice(hardwareMap, DcMotor.class, "up_down");
        relic = getDevice(hardwareMap, DcMotor.class, "relic");
        relic.setDirection(DcMotorSimple.Direction.FORWARD);
        relic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upDown.setDirection(DcMotorSimple.Direction.FORWARD);
        upDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm = getDevice(hardwareMap, Servo.class, "left_arm");
        rightArm = getDevice(hardwareMap, Servo.class, "right_arm");
        topLeftArm = getDevice(hardwareMap, Servo.class, "top_left_arm");
        topRightArm = getDevice(hardwareMap, Servo.class, "top_right_arm");
        
        lightsaberYawServo = getDevice(hardwareMap, Servo.class, "light_saber_yaw_servo");
        lightsaberPitchServo = getDevice(hardwareMap, Servo.class, "light_saber_pitch_servo");
        relicGrab = getDevice(hardwareMap, Servo.class, "relic_grab");
        relicRotate = getDevice(hardwareMap, Servo.class, "relicRotate");
        
        
        colorSensor = getDevice(hardwareMap, ColorSensor.class, "color");
        colorSensor.enableLed(true);
    }
}
