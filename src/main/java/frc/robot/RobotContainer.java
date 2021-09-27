// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberControllerCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ReleaseRatchetCommand;
import frc.robot.commands.ShooterOnCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.VisionAimingCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private HardwareMap hardwareMap = new HardwareMap();

        private XboxController m_driveController = hardwareMap.inputHardware.driveController;
        private OperatorBoard m_operatorController = hardwareMap.inputHardware.operatorBoard;

        private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(hardwareMap.shooterHardware);
        private SwerveDriveSubsystem m_swerveSubsystem = new SwerveDriveSubsystem(hardwareMap.swerveDriveHardware);
        private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(hardwareMap.intakeHardware);
        private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(hardwareMap.climberHardware);

        private SwerveJoystickCommand m_swerveJoystickCommand = new SwerveJoystickCommand(m_swerveSubsystem,
                        m_driveController);
        private MoveArmCommand m_moveArmCommand = new MoveArmCommand(m_operatorController, m_intakeSubsystem);
        private ClimberControllerCommand m_climberControllerCommand = new ClimberControllerCommand(m_climberSubsystem,
                        m_operatorController);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureButtonBindings();
                m_swerveSubsystem.setDefaultCommand(m_swerveJoystickCommand);
                m_intakeSubsystem.setDefaultCommand(m_moveArmCommand);
                m_climberSubsystem.setDefaultCommand(m_climberControllerCommand);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                // turns on shooter when Left Bumper is pressed
                m_operatorController.startShooter.toggleWhenPressed(
                                new ShooterOnCommand(m_shooterSubsystem, m_operatorController.startShooter));

                // turns on Feeder while button is held so balls will be shot
                m_operatorController.feeder
                                .whileHeld(new FeederCommand(m_shooterSubsystem, m_operatorController.feeder));

                // runs the intake while the button is held
                m_operatorController.intakeForward
                                .whileHeld(new IntakeCommand(m_intakeSubsystem, m_operatorController.intakeForward));

                // runs the intake backwards while the button is held
                m_operatorController.intakeReverse
                                .whileHeld(new OuttakeCommand(m_intakeSubsystem, m_operatorController.intakeReverse));

                // press to release ratchet. press again to lock ratchet.
                m_operatorController.releaseRatchet.toggleWhenPressed(
                                new ReleaseRatchetCommand(m_climberSubsystem, m_operatorController.releaseRatchet));

                // releases the Climber when button is pressed
                m_operatorController.releaseClimber.whenPressed(
                                new InstantCommand(m_climberSubsystem::releaseClimber, m_climberSubsystem));

                // set climber servo to reseting position
                m_operatorController.resetClimber.whenPressed(() -> m_climberSubsystem.resetClimber());

                // resets the gyro when the Start button is pressed
                new JoystickButton(m_driveController, Button.kStart.value)
                                .whenPressed(new InstantCommand(m_swerveSubsystem::resetGyro, m_swerveSubsystem));

                // Sets brake and coast mode with left bumper
                new JoystickButton(m_driveController, Button.kBumperLeft.value)
                                .whenPressed(() -> m_swerveSubsystem.setDriveIdleMode(IdleMode.kCoast))
                                .whenReleased(() -> m_swerveSubsystem.setDriveIdleMode(IdleMode.kBrake));

                new JoystickButton(m_driveController, Button.kA.value)
                                .whileHeld(new VisionAimingCommand(m_swerveSubsystem, m_driveController));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                return null;
        }
}
