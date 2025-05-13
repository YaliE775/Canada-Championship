package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JointConstants;
public class IntakeJoint extends SubsystemBase {

        private Telemetry telemetry;

        private final ServoEx rightAngle;
        private final ServoEx leftAngle;

        public IntakeJoint(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;

            rightAngle = new SimpleServo(hardwareMap, JointConstants.jointConstants.RIGHT_ANGLE_SERVO, 0, 1);
            leftAngle = new SimpleServo(hardwareMap, JointConstants.jointConstants.LEFT_ANGLE_SERVO, 0, 1);

            rightAngle.setInverted(true);
            leftAngle.setInverted(false);

        }

        public Command setAngle(double setAngle) {
            return new InstantCommand(() -> {
                rightAngle.setPosition(setAngle);
                leftAngle.setPosition(setAngle);
            }, this);
        }

        public Command autoDefaultCommand() {
            return new InstantCommand(() -> {
                rightAngle.setPosition(rightAngle.getPosition());
                leftAngle.setPosition(leftAngle.getPosition());
            },this).andThen(new WaitUntilCommand(() -> false));
        }

        public Command defaultCommand() {
            return new InstantCommand(() -> {
                rightAngle.setPosition(JointConstants.jointConstants.DEFAULT_POSITION2);
                leftAngle.setPosition(JointConstants.jointConstants.DEFAULT_POSITION2);
            },this).andThen(new WaitUntilCommand(() -> false));
        }

        public Command intakeSubmersible() {
            return new InstantCommand(() -> {
                rightAngle.setPosition(JointConstants.jointConstants.DEFAULT_POSITION);
                leftAngle.setPosition(JointConstants.jointConstants.DEFAULT_POSITION);
            },this).andThen(new WaitUntilCommand(() -> false));
        }

        @Override
        public void periodic() {
            telemetry.addData("manipulator angle is", rightAngle.getAngle());
            telemetry.addData("manipulator angle is", leftAngle.getAngle());
        }

    }


