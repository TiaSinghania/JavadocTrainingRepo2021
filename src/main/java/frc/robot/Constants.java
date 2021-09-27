/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Port numbers for the buttons on the operator board.
     */
    public static final class ControllerButtonPorts {
        public static final int BUTTON_ONE = 1;
        public static final int BUTTON_TWO = 2;
        public static final int BUTTON_THREE = 3;
        public static final int BUTTON_SIX = 6;
        public static final int BUTTON_ELEVEN = 11;
        public static final int BUTTON_TWELVE = 12;
        public static final int BUTTON_THIRTEEN = 13;
        public static final int BUTTON_FOURTEEN = 14;
    }

    /**
     * Port numbers for any hardware relating to swerve modules.
     */
    public static final class SwervePorts {
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 8;
        public static final int REAR_LEFT_DRIVE_MOTOR_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 4;
        public static final int REAR_RIGHT_DRIVE_MOTOR_PORT = 6;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int REAR_LEFT_TURNING_MOTOR_PORT = 3;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
        public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 7;

        public static final int FRONT_LEFT_TURNING_ENCODER_PORT = 0;
        public static final int REAR_LEFT_TURNING_ENCODER_PORT = 3;
        public static final int FRONT_RIGHT_TURNING_ENCODER_PORT = 1;
        public static final int REAR_RIGHT_TURNING_ENCODER_PORT = 2;
    }

    public static final class SwerveConstants {
        // previously 3.627
        public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 8.76;

        /** Distance between centers of front and back wheels on robot. */
        public static final double WHEEL_BASE = .67;

        /** Distance between right and left wheels on robot. */
        public static final double TRACK_WIDTH = .25 * 2;

        public static final double TRANSLATIONAL_FRICTION = 0.0205;

        // Swerve module turning encoder offsets
        public static final double FRONT_LEFT_ROTATION_OFFSET = 2.75 - (Math.PI / 5);
        public static final double BACK_LEFT_ROTATION_OFFSET = -6.091199;
        public static final double FRONT_RIGHT_ROTATION_OFFSET = 2.573;
        public static final double BACK_RIGHT_ROTATION_OFFSET = 3.9;

        // Gyro rotation rate deadzone
        public static final double GYRO_RATE_DEADZONE = 0.05;
        // TODO this value might need adjustment. copied from frc2020

        // Heading PID Controller
        public static final double HEADING_PID_P = 0.1;
        public static final double HEADING_PID_I = 0;
        public static final double HEADING_PID_D = 0;
        // TODO these values might need adjustment. copied from frc2020.

    }

    public static final class ShooterConstants {
        public static final int LEFT_FLYWHEEL_PORT = 16;
        public static final int RIGHT_FLYWHEEL_PORT = 17;

        public static final int KICKER_PORT = 26;
        public static final int WHEELS_PORT = 27;

        // TODO Might need a better, more descriptive name.
        // TODO write a comment explaining the distinction between these two values
        public static final double FLYWHEEL_RPM_THRESHOLD = 4700;
        public static final double FLYWHEEL_READY_RPM = 5000;
        // TODO Subject to tuning
    }

    public static final class IntakeConstants {
        // Speed [-1, 1] at which to run the intake motor.
        public static final double INTAKE_SPEED = 0.5;

        public static final int WHEELS_PORT = 25;
        public static final int ARM_PORT = 24;
    }

    /**
     * Servo positions for different climber states. Positions range between 0.0
     * (full left) and 1.0 (full right).
     */
    public static final class ClimberConstants {
        public static final double WINCH_RELEASE_SERVO_POSITION = 0.0;
        public static final double WINCH_LOCK_SERVO_POSITION = 0.5;
        public static final double SERVO_RELEASE_POSITION = 0.0;
        public static final double SERVO_RETURN_POSITION = 0.5;

        public static final double CLIMBER_CONTROL_SPEED_DEADZONE = 0.2;
    }

    public static final class ClimberPorts {
        public static final int SERVO_MOTOR_PORT = 1;
        public static final int RATCHET_SERVO_PORT = 0;
        public static final int WINCH_MOTOR_PORT = 19;
    }
}
