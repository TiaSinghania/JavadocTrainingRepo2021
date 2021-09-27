package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorBoard;
import frc.robot.Utils;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberControllerCommand extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;
    private OperatorBoard m_controller;

    public ClimberControllerCommand(ClimberSubsystem climberSubsystem, OperatorBoard operatorBoard) {
        m_climberSubsystem = climberSubsystem;
        m_controller = operatorBoard;
        addRequirements(m_climberSubsystem);
    }

    @Override
    public void execute() {
        m_climberSubsystem.climb(Utils.deadZones(-m_controller.getLeftJoystickY(),
                Constants.ClimberConstants.CLIMBER_CONTROL_SPEED_DEADZONE));
    }
}