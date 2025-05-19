package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class TelescopeConstants {
    @Config
    public static class LiftConstants {
        public static final String ELEVATOR_MOTOR = "Lift";

        public static double KP = 0.0045;
        public static double KI = 0.00000005;
        public static double KD = 0.00000001;

        public static double TICK_TO_CM = 0.020444444;

        public static double DEFAULT_POSITION = 200;
        public static double STARTING_HEIGHT = 0;

        public static double LOW_BASKET = 470;
        public static double HIGH_BASKET = 2420;

        public static double PRE_SCORE_HIGH_SPECIMEN = 1675;
        public static double SCORE_LOW_SPECIMEN = 0;
        public static double TOLERANCE = 15;
    }
}
