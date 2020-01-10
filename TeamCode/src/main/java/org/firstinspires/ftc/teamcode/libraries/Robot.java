package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.libraries.Constants.LEFT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MECANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.RIGHT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_ARM;

/*
 * Title: Robot
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This is the base library for any main op to be based off. It will contain all the
 *              motors, servos, and sensors.
 */

public class Robot {

    OpMode aOpMode;


    private LinearOpMode opMode;

    // Motors
    private DcMotor[] dcMotors = new DcMotor[8];

    //Servos
    private Servo[] servos = new Servo[7];

    // Sensors
//    private Rev2mDistanceSensor frontDistanceSensor;
//    private RevTouchSensor[] touchSensors = new RevTouchSensor[2];

    Robot(LinearOpMode opMode) {
        this.opMode = opMode;

        initDcMotors();
        initServos();
        initSensors();


    }

    private void initDcMotors() {
        //Naming our motors
        dcMotors[MOTOR_FRONT_LEFT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "frontLeftWheel");
        dcMotors[MOTOR_FRONT_RIGHT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "frontRightWheel");
        dcMotors[MOTOR_BACK_LEFT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "backLeftWheel");
        dcMotors[MOTOR_BACK_RIGHT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "backRightWheel");
        dcMotors[MOTOR_RIGHT_INTAKE] = opMode.hardwareMap.get(DcMotor.class, "rightIntake");
        dcMotors[MOTOR_LEFT_INTAKE] = opMode.hardwareMap.get(DcMotor.class, "leftIntake");
        dcMotors[MOTOR_ARM] = opMode.hardwareMap.get(DcMotor.class, "motorArm");


        dcMotors[MOTOR_FRONT_RIGHT_WHEEL].setDirection(DcMotorSimple.Direction.REVERSE);
        dcMotors[MOTOR_BACK_RIGHT_WHEEL].setDirection(DcMotorSimple.Direction.REVERSE);
    }


    private void initServos() {
        servos[SERVO_FOUNDATION1] = opMode.hardwareMap.get(Servo.class, "servoFoundation1");
        servos[SERVO_FOUNDATION2] = opMode.hardwareMap.get(Servo.class, "servoFoundation2");
        servos[SERVO_AUTONOMOUS_ARM] = opMode.hardwareMap.get(Servo.class, "servoAutonomousArm");
        servos[SERVO_AUTONOMOUS_GRABBER] = opMode.hardwareMap.get(Servo.class, "servoAutonomousGrabber");
        servos[SERVO_GRABBER] = opMode.hardwareMap.get(Servo.class, "servoGrabber");
        servos[SERVO_SCORING_ARM] = opMode.hardwareMap.get(Servo.class, "servoScoringArm");
        servos[SERVO_INTAKE] = opMode.hardwareMap.get(Servo.class, "servoIntake");

    }

    private void initSensors() {
//        frontDistanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
//        touchSensors[TOUCH_ARM_TOP] = opMode.hardwareMap.get(RevTouchSensor.class, "touchArmTop");
//        touchSensors[TOUCH_ARM_BOTTOM] = opMode.hardwareMap.get(RevTouchSensor.class, "touchArmBottom");
    }


    // Motor methods

    public boolean baseMotorsAreBusy() {
        return (dcMotors[MOTOR_FRONT_LEFT_WHEEL].isBusy() || dcMotors[MOTOR_BACK_RIGHT_WHEEL].isBusy() ||
                dcMotors[MOTOR_BACK_LEFT_WHEEL].isBusy() || dcMotors[MOTOR_BACK_RIGHT_WHEEL].isBusy());
    }

    public void setPower(int motorName, float power)
            throws InterruptedException {

        dcMotors[motorName].setPower(power);
    }


    public void runRobotToPosition(float fl_Power, float fr_Power, float bl_Power, float br_Power, int fl_Position, int fr_Position, int bl_Position, int br_Position,
                                   boolean isRampedPower) throws InterruptedException {
        double rampedPower;
        if (isRampedPower) {
            //sets the the power of all motors
            //since we are ramping up, start at the lowest power allowed.
            setPower(MOTOR_FRONT_LEFT_WHEEL, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(MOTOR_FRONT_RIGHT_WHEEL, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(MOTOR_BACK_LEFT_WHEEL, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(MOTOR_BACK_RIGHT_WHEEL, MOTOR_LOWER_POWER_THRESHOLD);

        } else {
            setPower(MOTOR_FRONT_LEFT_WHEEL, fl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(MOTOR_FRONT_RIGHT_WHEEL, fr_Power * RIGHT_MOTOR_TRIM_FACTOR);
            setPower(MOTOR_BACK_LEFT_WHEEL, bl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(MOTOR_BACK_RIGHT_WHEEL, br_Power * RIGHT_MOTOR_TRIM_FACTOR);
        }


        while (baseMotorsAreBusy()) {
            //wait until motors havce completed movement or timed out.
            //report motor positions for debugging

            //adjust the motor speeds by adjusting Power proportional to distance that needs to be travelled.


            //Ramped Move block formula:
            //RP=PMax(1-4*(0.5-DT/DD)^2)
            //where RP=Ramped Power, PMax is maximum power available, DT=Distance Travelled, DD=Distance to be travelled
            //fl_position (target for the front left motor in encoder clicks can be taken as the proxy for all motors.

            //using fl_power as proxy for all wheel power, the sign is not relevant in this runmode.

            float rampedPowerRaw = (float) (fl_Power * (1 - 4 * (Math.pow((0.5f -
                    Math.abs((dcMotors[MOTOR_FRONT_LEFT_WHEEL].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f))));

            //use another variable to check and adjust power limits, so we can display raw power values.
            if (isRampedPower) {
                rampedPower = rampedPowerRaw;
            } else {
                rampedPower = fl_Power; //as proxy for all power.
            }

            if (Math.signum(fl_Position) != Math.signum(fr_Position)) {
                //we are moving sideways, since the front left and right wheels are rotating in opposite directions
                //we should check against sideways limits.
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
                }
            } else {
                //else we are moving forward
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_FB_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_FB_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
                }
            }
        }
    }


    void setDcMotorPower(int index, float power) {
        dcMotors[index].setPower(power);
    }

    void setDcMotorMode(int index, DcMotor.RunMode runMode) {
        dcMotors[index].setMode(runMode);
    }

    int getDcMotorPosition(int index) {
        return dcMotors[index].getCurrentPosition();
    }

    void setDcMotorTargetPosition(int index, int targetPosition) {
        dcMotors[index].setTargetPosition(targetPosition);
    }

    boolean isMotorBusy(int index) {
        return dcMotors[index].isBusy();
    }

    // Servo methods
    void setServoPosition(int index, float position) {
        servos[index].setPosition(position);
    }


    void setDeltaServoPosition(int index, float delta) {
        servos[index].setPosition(
                // This makes sure the servo positions are between 0 and 1
                Range.clip(servos[index].getPosition() + delta, 0, 1));
    }

    float getServoPosition(int index) {
        return (float) servos[index].getPosition();
    }
//    boolean isTouchSensorPressed(int index) {
//        return touchSensors[index].isPressed();
//    }

//    double getWallDistanceCenti() {
//        return (frontDistanceSensor.getDistance(DistanceUnit.METER) * 100);
//    }
}