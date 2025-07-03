package org.firstinspires.ftc.teamcode.sybsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Telescope extends SubsystemBase {
    private final Telemetry telemetry;
    private final DcMotorEx elevatorMotor;

    private final PIDController pid = new PIDController(
            Constants.TelescopeConstants.KP,
            Constants.TelescopeConstants.KI,
            Constants.TelescopeConstants.KD
    );

    private double currentPosition;

    public Telescope(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        elevatorMotor = hardwareMap.get(DcMotorEx.class,Constants.TelescopeConstants.ELEVATOR_MOTOR);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pid.setTolerance(Constants.TelescopeConstants.TOLERANCE);

    }

    public void update() {
        double output = pid.calculate(elevatorMotor.getCurrentPosition());
        elevatorMotor.setPower(output);

    }

    public Command setSetPoint(double setPoint) {
        return new InstantCommand(() -> pid.setSetPoint(setPoint),this);

    }

    public Command defaultCommand() {
        return new RunCommand(() -> pid.setSetPoint(Constants.TelescopeConstants.DEFAULT_POSITION),this);
    }

    @Override
    public void periodic() {
        currentPosition = elevatorMotor.getCurrentPosition();

        telemetry.addData("Lift SetPoint", pid.getSetPoint());
        telemetry.addData("Lift Position", elevatorMotor.getCurrentPosition());

        pid.setSetPoint(pid.getSetPoint());
        telemetry.update();

    }

}
