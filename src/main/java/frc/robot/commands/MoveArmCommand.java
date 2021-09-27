// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorBoard;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveArmCommand extends CommandBase {
  private OperatorBoard m_controller;
  private IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new MoveArmCommand.
   */
  public MoveArmCommand(OperatorBoard controller, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // joystickOutput will be between 1 and -1
    double joystickOutput = m_controller.getRightJoystickY();
    m_intakeSubsystem.moveArm(-0.75 * joystickOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
