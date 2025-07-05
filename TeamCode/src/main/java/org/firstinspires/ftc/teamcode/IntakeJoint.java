package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeJoint extends SubsystemBase {
    private Telemetry telemetry;

    private final ServoEx rightAngle;
    private final ServoEx leftAngle;

    public IntakeJoint(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightAngle = new SimpleServo(hardwareMap, Constants.IntakeConstants.RIGHT_ANGLE_SERVO, 0, 1);
        leftAngle = new SimpleServo(hardwareMap, Constants.IntakeConstants.LEFT_ANGLE_SERVO, 0, 1);

        rightAngle.setInverted(true);
        leftAngle.setInverted(false);

    }

    public Command setAngle(double setAngle) {
        return new InstantCommand(() -> {
            rightAngle.setPosition(setAngle);
            leftAngle.setPosition(setAngle);
        }, this);
    }
    //TODO: ADD CORRECT VARIABLES
    public Command autoDefaultCommand() {
        return setAngle(1).andThen(new WaitUntilCommand(() -> false));
    }

    public Command defaultCommand() {
        return setAngle(1).andThen(new WaitUntilCommand(() -> false));
    }

    public Command intakeSubmersible() {
        return setAngle(1).andThen(new WaitUntilCommand(() -> false));
    }

    @Override
    public void periodic() {
        telemetry.addData("manipulator angle ", rightAngle.getAngle());
        telemetry.addData("manipulator angle ", leftAngle.getAngle());
    }
}
