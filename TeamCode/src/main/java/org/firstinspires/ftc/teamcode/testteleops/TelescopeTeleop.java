package org.firstinspires.ftc.teamcode.testteleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.NewJoint;

@TeleOp
public class TelescopeTeleop extends CommandOpMode {
        @Override
        public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            GamepadEx controller = new GamepadEx(gamepad1);
            NewJoint newJoint = new NewJoint(telemetry, hardwareMap);

            controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(newJoint.active());
        }
    }

