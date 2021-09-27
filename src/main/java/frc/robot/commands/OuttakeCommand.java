/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard.OperatorBoardButton;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private OperatorBoardButton m_button;

  /**
   * Creates a new OuttakeCommand.
   */

  public OuttakeCommand(IntakeSubsystem intake, OperatorBoardButton button) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);

    m_button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_intakeSubsystem.outtake();
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
}
