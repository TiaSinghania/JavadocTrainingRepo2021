// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase implements AutoCloseable {

  private IntakeSubsystem m_intakeSubsystem;
  private OperatorBoardButton m_button;

  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(IntakeSubsystem intake, OperatorBoardButton button) {

    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);

    m_button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intake();
    m_button.turnLightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_button.turnLightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void close() throws Exception {

    // Do this soon

  }
}
