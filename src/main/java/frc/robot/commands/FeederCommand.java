// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  private OperatorBoardButton m_button;


  public FeederCommand(ShooterSubsystem shooter, OperatorBoardButton button) {
    m_shooterSubsystem = shooter;
    m_button = button;
  }

  @Override
  public void initialize() {
    m_button.turnLightOn();
  }

  @Override
  public void execute() {
    m_shooterSubsystem.turnFeederOn();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.turnFeederOff();
    m_button.turnLightOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
