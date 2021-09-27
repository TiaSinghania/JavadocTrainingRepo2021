// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOnCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private OperatorBoardButton m_button;

  /** Creates a new ShooterCommand. */
  public ShooterOnCommand(ShooterSubsystem shooterSubsystem, OperatorBoardButton button) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    m_button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("Shooter On command triggered", false);
    m_button.turnLightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentSpeed = m_shooterSubsystem.getFlywheelRPM();
    if (currentSpeed < Constants.ShooterConstants.FLYWHEEL_RPM_THRESHOLD) { // TODO subject to tuning
      m_shooterSubsystem.setFlywheelPower(1);
    }

    else if (currentSpeed > Constants.ShooterConstants.FLYWHEEL_RPM_THRESHOLD) {
      m_shooterSubsystem.setFlywheelPower(0.93);
      // TODO put this number in Constants
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFlywheelPower(0);
    m_button.turnLightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
