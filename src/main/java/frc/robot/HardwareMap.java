package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveModule;

public class HardwareMap {

        /**
         * Constructs the controllers.
         */
        public class InputHardware {
                public XboxController driveController;
                // public XboxController operatorController;
                public OperatorBoard operatorBoard;

                public InputHardware() {
                        driveController = new XboxController(0);
                        // operatorController = new XboxController(1);
                        operatorBoard = new OperatorBoard(1);
                }
        }

        /**
         * Constructs the drive motors, turning motors and turning encoders for each
         * module
         */
        private class SwerveModuleHardware {
                public CANSparkMax frontLeftDriveMotor;
                public CANSparkMax frontRightDriveMotor;
                public CANSparkMax backLeftDriveMotor;
                public CANSparkMax backRightDriveMotor;

                public CANSparkMax frontLeftTurningMotor;
                public CANSparkMax frontRightTurningMotor;
                public CANSparkMax backLeftTurningMotor;
                public CANSparkMax backRightTurningMotor;

                public AbsoluteEncoder frontLeftTurningEncoder;
                public AbsoluteEncoder frontRightTurningEncoder;
                public AbsoluteEncoder backLeftTurningEncoder;
                public AbsoluteEncoder backRightTurningEncoder;

                public SwerveModuleHardware() {
                        frontLeftDriveMotor = new CANSparkMax(Constants.SwervePorts.FRONT_LEFT_DRIVE_MOTOR_PORT,
                                        MotorType.kBrushless);
                        frontRightDriveMotor = new CANSparkMax(Constants.SwervePorts.FRONT_RIGHT_DRIVE_MOTOR_PORT,
                                        MotorType.kBrushless);
                        backLeftDriveMotor = new CANSparkMax(Constants.SwervePorts.REAR_LEFT_DRIVE_MOTOR_PORT,
                                        MotorType.kBrushless);
                        backRightDriveMotor = new CANSparkMax(Constants.SwervePorts.REAR_RIGHT_DRIVE_MOTOR_PORT,
                                        MotorType.kBrushless);

                        frontLeftTurningMotor = new CANSparkMax(Constants.SwervePorts.FRONT_LEFT_TURNING_MOTOR_PORT,
                                        MotorType.kBrushless);
                        frontRightTurningMotor = new CANSparkMax(Constants.SwervePorts.FRONT_RIGHT_TURNING_MOTOR_PORT,
                                        MotorType.kBrushless);
                        backLeftTurningMotor = new CANSparkMax(Constants.SwervePorts.REAR_LEFT_TURNING_MOTOR_PORT,
                                        MotorType.kBrushless);
                        backRightTurningMotor = new CANSparkMax(Constants.SwervePorts.REAR_RIGHT_TURNING_MOTOR_PORT,
                                        MotorType.kBrushless);

                        frontLeftTurningEncoder = new AbsoluteEncoder(
                                        Constants.SwervePorts.FRONT_LEFT_TURNING_ENCODER_PORT, true,
                                        Constants.SwerveConstants.FRONT_LEFT_ROTATION_OFFSET);
                        frontRightTurningEncoder = new AbsoluteEncoder(
                                        Constants.SwervePorts.FRONT_RIGHT_TURNING_ENCODER_PORT, true,
                                        Constants.SwerveConstants.FRONT_RIGHT_ROTATION_OFFSET);
                        backLeftTurningEncoder = new AbsoluteEncoder(
                                        Constants.SwervePorts.REAR_LEFT_TURNING_ENCODER_PORT, true,
                                        Constants.SwerveConstants.BACK_LEFT_ROTATION_OFFSET);
                        backRightTurningEncoder = new AbsoluteEncoder(
                                        Constants.SwervePorts.REAR_RIGHT_TURNING_ENCODER_PORT, true,
                                        Constants.SwerveConstants.BACK_RIGHT_ROTATION_OFFSET);
                }
        }

        /**
         * Constructs hardware for the feeder and flywheel system.
         */
        public class ShooterHardware {
                // encoder for the flywheel
                public CANEncoder rightCanEncoder;
                // motors to run the flywheel, combined in a speedcontrollergroup
                private CANSparkMax leftFlywheel;
                private CANSparkMax rightFlywheel;
                public SpeedControllerGroup flywheel;
                // motors for the feeder system, combined in a speedcontrollergroup
                private WPI_VictorSPX wheels;
                private WPI_VictorSPX kicker;
                public SpeedControllerGroup feeder;

                public ShooterHardware() {
                        leftFlywheel = new CANSparkMax(Constants.ShooterConstants.LEFT_FLYWHEEL_PORT,
                                        MotorType.kBrushless);
                        leftFlywheel.setInverted(true);
                        rightFlywheel = new CANSparkMax(Constants.ShooterConstants.RIGHT_FLYWHEEL_PORT,
                                        MotorType.kBrushless);
                        rightCanEncoder = rightFlywheel.getEncoder();
                        flywheel = new SpeedControllerGroup(leftFlywheel, rightFlywheel);

                        kicker = new WPI_VictorSPX(Constants.ShooterConstants.KICKER_PORT);
                        wheels = new WPI_VictorSPX(Constants.ShooterConstants.WHEELS_PORT);
                        wheels.setInverted(true);
                        feeder = new SpeedControllerGroup(kicker, wheels);
                }
        };

        /**
         * Constructs motors for arm and intake
         */
        public class IntakeHardware {
                public VictorSPX intakeController;
                public VictorSPX armController;

                public IntakeHardware() {
                        intakeController = new VictorSPX(25);
                        armController = new VictorSPX(24);
                        armController.setNeutralMode(NeutralMode.Brake);
                }
        }

        /**
         * Constructs hardware for the whole drivetrain (gyro and each swervemodule)
         */
        public class SwerveDriveHardware {
                private double x;
                private double y;
                private SwerveModuleHardware swerveModuleHardware;
                public SwerveModule frontLeft;
                public SwerveModule frontRight;
                public SwerveModule backLeft;
                public SwerveModule backRight;

                public AHRS gyro;

                public SwerveDriveHardware() {
                        x = Constants.SwerveConstants.WHEEL_BASE / 2;
                        y = Constants.SwerveConstants.TRACK_WIDTH / 2;
                        swerveModuleHardware = new SwerveModuleHardware();
                        frontLeft = new SwerveModule(swerveModuleHardware.frontLeftDriveMotor,
                                        swerveModuleHardware.frontLeftTurningMotor, x, y,
                                        swerveModuleHardware.frontLeftTurningEncoder);
                        frontRight = new SwerveModule(swerveModuleHardware.frontRightDriveMotor,
                                        swerveModuleHardware.frontRightTurningMotor, x, -y,
                                        swerveModuleHardware.frontRightTurningEncoder);
                        backLeft = new SwerveModule(swerveModuleHardware.backLeftDriveMotor,
                                        swerveModuleHardware.backLeftTurningMotor, -x, y,
                                        swerveModuleHardware.backLeftTurningEncoder);
                        backRight = new SwerveModule(swerveModuleHardware.backRightDriveMotor,
                                        swerveModuleHardware.backRightTurningMotor, -x, -y,
                                        swerveModuleHardware.backRightTurningEncoder);

                        gyro = new AHRS();
                }
        }

        /**
         * Constructs servos and motor for climber
         */
        public class ClimberHardware {
                // pulls the pin to release the climber
                public Servo servoMotor;
                // locks or unlocks the climber so it can extend only when needed
                public Servo ratchetServo;
                // actually extends and retracts the climber
                public CANSparkMax winchMotor;

                public ClimberHardware() {
                        servoMotor = new Servo(Constants.ClimberPorts.SERVO_MOTOR_PORT);
                        ratchetServo = new Servo(Constants.ClimberPorts.RATCHET_SERVO_PORT);
                        winchMotor = new CANSparkMax(Constants.ClimberPorts.WINCH_MOTOR_PORT, MotorType.kBrushless);
                }
        }

        public InputHardware inputHardware;
        public ShooterHardware shooterHardware;

        public SwerveDriveHardware swerveDriveHardware;

        public IntakeHardware intakeHardware;
        public ClimberHardware climberHardware;

        public HardwareMap() {
                inputHardware = new InputHardware();
                shooterHardware = new ShooterHardware();

                swerveDriveHardware = new SwerveDriveHardware();

                intakeHardware = new IntakeHardware();
                climberHardware = new ClimberHardware();
        }
}
