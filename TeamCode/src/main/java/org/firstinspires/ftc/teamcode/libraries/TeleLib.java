package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.libraries.Constants.GAMEPAD_JOYSTICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.GAMEPAD_TRIGGER_TOLERANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_TAPE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_UP_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_CAPSTONE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_CAPSTONE_DROP;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_CAPSTONE_HOLD;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER_REST;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_EXTEND;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_RETRACT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_STOPPER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_STOPPER_STOP;

/*
 * Title: TeleLib
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This will contain the methods for TeleOp, and other TeleOp-related programs.
 */

public class TeleLib {
    private Robot robot;
    private LinearOpMode opMode;

    private ElapsedTime latcherServoInputDelay;
    private ElapsedTime scoringServoInputDelay;
    private ElapsedTime intakeAngleServoInputDelay;
    private ElapsedTime servoArmInputDelay;

    public TeleLib(LinearOpMode opMode) {
        robot = new Robot(opMode);
        this.opMode = opMode;

        opMode.gamepad1.setJoystickDeadzone(GAMEPAD_JOYSTICK_TOLERANCE);
        opMode.gamepad2.setJoystickDeadzone(GAMEPAD_JOYSTICK_TOLERANCE);

//        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);

        latcherServoInputDelay = new ElapsedTime();
        scoringServoInputDelay = new ElapsedTime();
        intakeAngleServoInputDelay = new ElapsedTime();
    }

    //gamepad1

    public void processDrive() {
        // Values need to be reversed (up on joystick is -1)
        double r = Math.hypot(opMode.gamepad1.left_stick_x, -opMode.gamepad1.left_stick_y);  //y ish changed to positive
        double robotAngle = Math.atan2(-opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = opMode.gamepad1.right_stick_x;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, (float) (r * Math.cos(robotAngle) + rightX));
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, (float) (r * Math.sin(robotAngle) - rightX));
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, (float) (r * Math.sin(robotAngle) + rightX));
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, (float) (r * Math.cos(robotAngle) - rightX));

        float powerFactor = 1;
        boolean isDPadPressed = true;

        if (opMode.gamepad1.dpad_up && isDPadPressed) {
            powerFactor = 3;
            robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, (float) (r * Math.cos(robotAngle) + rightX) / powerFactor);
            robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, (float) (r * Math.sin(robotAngle) - rightX) / powerFactor);
            robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, (float) (r * Math.sin(robotAngle) + rightX) / powerFactor);
            robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, (float) (r * Math.cos(robotAngle) - rightX) / powerFactor);
        }

        if (opMode.gamepad1.dpad_down && isDPadPressed) {
            robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, (float) (r * Math.cos(robotAngle) + rightX));
            robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, (float) (r * Math.sin(robotAngle) - rightX));
            robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, (float) (r * Math.sin(robotAngle) + rightX));
            robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, (float) (r * Math.cos(robotAngle) - rightX));
        }
    }

    public void processOutakeStone() {
        if (opMode.gamepad1.left_bumper) {
            robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, .25f);
            robot.setDcMotorPower(MOTOR_LEFT_INTAKE, -.25f);
            robot.setServoPosition(SERVO_INTAKE, 1);
        }
    }

    public void processFoundation() {
        if (opMode.gamepad1.a) {
            robot.setServoPosition(SERVO_FOUNDATION1, -SERVO_FOUNDATION_GRAB1);
            robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_GRAB2);

        } else if (opMode.gamepad1.b) {
            robot.setServoPosition(SERVO_FOUNDATION1, SERVO_FOUNDATION_REST1);
            robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_REST2);

        }
    }

    public void processStopIntake() {
        if (opMode.gamepad1.y) {    //|| !isBlockInIntake()
            robot.setDcMotorPower(MOTOR_LEFT_INTAKE, 0);
            robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, 0);
            robot.setServoPosition(SERVO_INTAKE, 0.5f);
        }
    }

//    private boolean isBlockInIntake() {
//        if (distance <= 2) {
//            return true;
//        } else {
//            return false;
//        }
//
//        return distance <= 2;
//    }

    public void processIntakeStone() {
        if (opMode.gamepad1.right_bumper) {
            robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, -.25f);
            robot.setDcMotorPower(MOTOR_LEFT_INTAKE, .25f);
            robot.setServoPosition(SERVO_INTAKE, 0);
        }
    }

    public void processTapeMeasure() {
        if (opMode.gamepad1.right_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
            // Extend
            robot.setDcMotorPower(MOTOR_TAPE, -.75f);
        } else if (opMode.gamepad1.left_trigger > GAMEPAD_TRIGGER_TOLERANCE) {
            // Retract
            robot.setDcMotorPower(MOTOR_TAPE, .75f);
        } else {
            robot.setDcMotorPower(MOTOR_TAPE, 0);
        }

    }


    //gamepad 2

    public void processMoveArm() {
        if (opMode.gamepad2.right_bumper) {
            // Extend
            robot.setDcMotorPower(MOTOR_ARM, 1f);
        } else if (opMode.gamepad2.left_bumper) {
            // Retract
            robot.setDcMotorPower(MOTOR_ARM, -1f);
        } else {
            robot.setDcMotorPower(MOTOR_ARM, 0);
        }
    }

    public void processDropCapstone() {
        boolean isDPadPressed = true;

        if (opMode.gamepad2.dpad_down && isDPadPressed) {
            robot.setServoPosition(SERVO_CAPSTONE, SERVO_CAPSTONE_DROP);
        }
        if (opMode.gamepad2.dpad_up && isDPadPressed) {
            robot.setServoPosition(SERVO_CAPSTONE, SERVO_CAPSTONE_HOLD);
        }
    }

    public void processExtendArm() {
        if (opMode.gamepad2.y) {
            // Extend
            robot.setServoPosition(SERVO_SCORING_ARM, SERVO_SCORING_EXTEND);
        }
    }

    public void processRetractArm() {
        if (opMode.gamepad2.x) {
            // Retract
            robot.setServoPosition(SERVO_SCORING_ARM, SERVO_SCORING_RETRACT);
        }
    }

    public void processGrabStone() {
        if (opMode.gamepad2.a) {
            robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
        }
    }

    public void processScoreStone() {
        if (opMode.gamepad2.b) {
            robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);
        }
    }

    public void restServoStopper() {
        robot.setServoPosition(SERVO_STOPPER, SERVO_STOPPER_STOP);
    }

    public void autonomousArmUp() {
        robot.setServoPosition(SERVO_AUTONOMOUS_ARM, SERVO_AUTONOMOUS_UP_ARM);
    }

}
//
//    public void processScoreStone() {
//        if (opMode.gamepad2.x) {
//            robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_RECIEVE);
//        }
//    }
//
//    public void processServoGrab() {
//        if (opMode.gamepad2.a) {
//            robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
//        } else if (opMode.gamepad2.b) {
//            robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);
//        }
//    }
//
//    public void processServoArm() {
//        if (opMode.gamepad2.y) {
//            robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_SCORE);
//        }
//    }


//        if (opMode.gamepad1.right_bumper && servoArmInputDelay.seconds() > .25)
//            if (robot.getServoPosition(SERVO_ARM) == SERVO_ARM_POS_RECIEVE) {
//                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_RECIEVE);
//            } else {
//                robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_SCORE);
//            }
//        latcherServoInputDelay.reset();
//        if (opMode.gamepad1.dpad_up && scoringServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_ARM, .02f);
//            scoringServoInputDelay.reset();
//        } else if (opMode.gamepad1.dpad_down && scoringServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_ARM, -.02f);
//            scoringServoInputDelay.reset();
//        }
//    }

//        if (opMode.gamepad2.dpad_up && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_FOUNDATION1, .02f);
//            intakeAngleServoInputDelay.reset();
//        } else if (opMode.gamepad2.dpad_down && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_FOUNDATION2, -.02f);
//            intakeAngleServoInputDelay.reset();
//        }
//    }

//        if (opMode.gamepad1.dpad_up && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_GRABBER, .02f);
//            intakeAngleServoInputDelay.reset();
//        } else if (opMode.gamepad1.dpad_down && intakeAngleServoInputDelay.seconds() > .2f) {
//            robot.setDeltaServoPosition(SERVO_GRABBER, -.02f);
//            intakeAngleServoInputDelay.reset();
//        }






