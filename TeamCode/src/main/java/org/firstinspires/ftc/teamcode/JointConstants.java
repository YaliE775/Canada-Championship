package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class JointConstants {
    @Config
    public static class jointConstants {
        public static final String RIGHT_ANGLE_SERVO = "Angle Right";
        public static final String RIGHT_ROLLER = "Roller Right";
        public static final String LEFT_ANGLE_SERVO = "Angle Left";
        public static final String LEFT_ROLLER = "Roller Left";
        public static final String DISTANCE_SENSOR = "color sensor";

        public static double INTAKE_POWER = 0.9;
        public static double OUTTAKE_POWER = -0.6;
        public static double ZERO_POWER = 0;

        public static double DEFAULT_POSITION = 0.1;
        public static double DEFAULT_POSITION2 = 0.5;
        public static double INTAKE_DEFAULT = 1;
        public static double INTAKE_DEFAULT2 = 0.1;

        public static double PRE_SCORE_SPECIMEN_LOW = 0.5;
        public static double PRE_SCORE_SPECIMEN_HIGH = 0;

        public static double OBSERVATION_OUTTAKE = 0.5;

        public static double INTAKE_SPECIMEN_POSITION = 0.55;
        public static double INTAKE_SAMPLE_POSITION = 0.9;
        public static double EXIT_SUBMERSIBLE = 0.3;

        public static double SCORE_BASKET_LOW = 1;
        public static double DEFAULT_INTAKE = 0.15;
        public static double SCORE_BASKET_HIGH = 0.35;

        public static double SCORE_SPECIMEN_LOW = 0.5;
        public static double SCORE_SPECIMEN_HIGH = 0.9;
    }
}
