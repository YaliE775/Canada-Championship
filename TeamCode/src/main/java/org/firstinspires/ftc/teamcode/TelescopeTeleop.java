package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TelescopeTeleop extends CommandOpMode {
        @Override
        public void initialize() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            GamepadEx controller = new GamepadEx(gamepad1);
            Telescope telescope = new Telescope(telemetry, hardwareMap);

            telescope.setDefaultCommand(telescope.defaultCommand());

            controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(telescope.setSetPoint(TelescopeConstants.telescopeConstants.LOW_BASKET));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(telescope.setSetPoint(TelescopeConstants.telescopeConstants.HIGH_BASKET));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileActiveContinuous(telescope.setSetPoint(TelescopeConstants.telescopeConstants.SCORE_LOW_SPECIMEN));
            controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileActiveContinuous(telescope.setSetPoint(TelescopeConstants.telescopeConstants.PRE_SCORE_HIGH_SPECIMEN));
        }
    }

