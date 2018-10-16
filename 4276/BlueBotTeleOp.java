package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.marvframework.utils.MarvHardwareMap;
import org.firstinspires.ftc.teamcode.marvframework.MarvTeleOp;

/**
 * Created by andy on 11/2/17, edited by Benjamin on 1/15/17.
 */

@TeleOp(name = "BlueBotTeleOp", group = "BlueBot")
public class BlueBotTeleOp extends MarvTeleOp {

    private BlueBotHardware hardware = BlueBotHardware.getHardware();

    //We don't want local variables in a loop
    private float inputLeftWheels = 0, inputRightWheels = 0, inputRelic = 0;
    private float inputRightArm = 0, inputLeftArm = 0, inputTopLeftArm = 0, inputTopRightArm = 0, inputRelicg = 0, inputRelicr = 0;
    

    //Servo numbers
    private final float left_servo_min = 0.10f, left_servo_max = 0.90f;
    private final float right_servo_min = 0.10f, right_servo_max = 0.90f;
    private final float top_left_servo_min = 0.10f, top_left_servo_max = 0.90f;
    private final float top_right_servo_min = 0.10f, top_right_servo_max = 0.90f;
    private final float relicg_servo_min = 0.10f, relicg_servo_max = 0.90f;
    private final float relicr_servo_min = 0.01f, relicr_servo_max = 0.99f;

    private float servoLeftPosition = 0, servoRightPosition = 0, servoTopLeftPosition = 0, servoTopRightPosition = 0, relicgPosition = 0, relicrPosition = 0;
    
    //Speeds
    private static final float wheel_speed = 1.0f;
    private static final float servo_speed = 0.013f;
    private static final float lift_speed = 1.0f;
    private static final float relic_speed = 1.0f;

    @Override
    public MarvHardwareMap getHardwareMap() {
        return hardware;
    }

    public void initTeleOp() { }

    @Override
    public void loop() {
        //Wheels
        if (Math.abs(gamepad1.left_stick_y) > 0.02 || Math.abs(gamepad1.right_stick_y) > 0.02) {
        inputLeftWheels = -gamepad1.left_stick_y * wheel_speed;
        inputRightWheels = -gamepad1.right_stick_y * wheel_speed;
        } else if (gamepad1.dpad_up) {
            inputRightWheels = .2f;
            inputLeftWheels = .2f;
        } else if (gamepad1.dpad_down) {
            inputRightWheels = -.2f;
            inputLeftWheels = -.2f;
        } else if (gamepad1.dpad_right) {
            inputRightWheels = -.4f;
            inputLeftWheels = .4f;
        } else if (gamepad1.dpad_left) {
            inputRightWheels = .4f;
            inputLeftWheels = -.4f;
        }
        else {
            inputRightWheels = 0f;
            inputLeftWheels = 0f;
        }
        hardware.leftWheel.setPower(inputLeftWheels);
        hardware.rightWheel.setPower(inputRightWheels);

        //Lift
        if (gamepad2.left_trigger > .5) {
            hardware.upDown.setPower(lift_speed);
        } 
        else if (gamepad2.right_trigger > .5) {
            hardware.upDown.setPower(-lift_speed);
        } 
        else{
            hardware.upDown.setPower(0f);
        }
        
        //Arms
        inputLeftArm = -gamepad2.left_stick_x;
        inputRightArm = gamepad2.right_stick_x;
        inputTopLeftArm = -gamepad2.left_stick_x;
        inputTopRightArm = gamepad2.right_stick_x;
        if (gamepad2.a) inputRelicr = 1;
        else if (gamepad2.y) inputRelicr = -1;
        else inputRelicr = 0;
        if (gamepad2.left_bumper) inputRelicg = 1;
        else if (gamepad2.right_bumper) inputRelicg = -1;
        else inputRelicg = 0;
        
        if (gamepad2.dpad_up) inputRelic = .6f;
        else if (gamepad2.dpad_down) inputRelic = -.6f;
        else inputRelic = 0f;
        hardware.relic.setPower(inputRelic);
        

        servoLeftPosition = (float) Math.max(left_servo_min, Math.min(left_servo_max, servoLeftPosition - inputLeftArm * servo_speed));
        servoRightPosition = (float) Math.max(right_servo_min, Math.min(right_servo_max, servoRightPosition + inputRightArm * servo_speed));
        servoTopLeftPosition = (float) Math.max(top_left_servo_min, Math.min(top_left_servo_max, servoTopLeftPosition - inputTopLeftArm * servo_speed));
        servoTopRightPosition = (float) Math.max(top_right_servo_min, Math.min(top_right_servo_max, servoTopRightPosition + inputTopRightArm * servo_speed));
        relicgPosition = (float) Math.max(relicg_servo_min, Math.min(relicg_servo_max, relicgPosition + inputRelicg * servo_speed));
        relicrPosition = (float) Math.max(relicr_servo_min, Math.min(relicr_servo_max, relicrPosition + inputRelicr * servo_speed));
        
        hardware.relicGrab.setPosition(relicgPosition);
        hardware.relicRotate.setPosition(relicrPosition);
        hardware.leftArm.setPosition(servoLeftPosition);
        hardware.rightArm.setPosition(servoRightPosition);
        hardware.topLeftArm.setPosition(servoTopLeftPosition);
        hardware.topRightArm.setPosition(servoTopRightPosition);
        hardware.lightsaberPitchServo.setPosition(1.50f);
        
       // telemetry.addLine().addData("left", servoLeftPosition).addData("right", servoRightPosition).addData("top_left", servoTopLeftPosition).addData("top_right", servoTopRightPosition);
    }

    @Override
    public void stop() {
        hardware.leftWheel.setPower(0);
        hardware.rightWheel.setPower(0);
        hardware.upDown.setPower(0);
        hardware.relic.setPower(0);
    }
}
