package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Telescope  extends SubsystemBase {
        private final Telemetry telemetry;
        private final DcMotorEx elevatorMotor;

        private final PIDController pid = new PIDController(
                TelescopeConstants.telescopeConstants.KP,
                TelescopeConstants.telescopeConstants.KI,
                TelescopeConstants.telescopeConstants.KD
        );

        public static boolean isUsingPID = true;
        private static double currentPosition;

        public Telescope(Telemetry telemetry, HardwareMap hardwareMap) {
            this.telemetry = telemetry;
            elevatorMotor = hardwareMap.get(DcMotorEx.class, TelescopeConstants.telescopeConstants.ELEVATOR_MOTOR);
            elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            pid.setTolerance(TelescopeConstants.telescopeConstants.TOLERANCE);
        }

        public void update() {
            double output = pid.calculate(elevatorMotor.getCurrentPosition());
            elevatorMotor.setPower(output);
        }

        public Command setSetPoint(double setPoint) {
            return new InstantCommand(() -> pid.setSetPoint(setPoint), this);
        }

        public Command defaultCommand() {
            return new RunCommand(() -> pid.setSetPoint(TelescopeConstants.telescopeConstants.DEFAULT_POSITION), this);
        }

        public static double getHeight() {
            return (currentPosition * TelescopeConstants.telescopeConstants.TICK_TO_CM) + TelescopeConstants.telescopeConstants.STARTING_HEIGHT;
        }

        @Override
        public void periodic() {
            currentPosition = elevatorMotor.getCurrentPosition();

            telemetry.addData("Lift SetPoint", pid.getSetPoint());
            telemetry.addData("Lift Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Lift Height", getHeight());
            telemetry.update();
            pid.setSetPoint(pid.getSetPoint());

        }

    }

