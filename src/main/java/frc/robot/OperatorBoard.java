package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A wrapper class for our custom-made operator board. Does not have a
 * constructor. Access statically. Access Buttons via public static state.
 * Access axis values via methods. Use methods to make buttons light up.
 */
public class OperatorBoard {

    private static Joystick HARDWARE;
    public OperatorBoardButton startShooter;
    public OperatorBoardButton feeder;
    public OperatorBoardButton intakeForward;
    public OperatorBoardButton intakeReverse;
    public OperatorBoardButton releaseRatchet;
    public OperatorBoardButton releaseClimber;
    public OperatorBoardButton resetClimber;
    public OperatorBoardButton BUTTON_TWELVE;

    /**
     * Constructs one operator board. Since we only have one operator board, this
     * class should be treated as a singleton.
     * 
     * @param port The USB port number for the controller
     */
    public OperatorBoard(int port) {
        HARDWARE = new Joystick(port);

        startShooter = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_SIX);
        feeder = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_TWO);
        intakeForward = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_THREE);
        intakeReverse = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_ONE);
        releaseRatchet = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_FOURTEEN);
        releaseClimber = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_THIRTEEN);
        resetClimber = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_ELEVEN);
        BUTTON_TWELVE = new OperatorBoardButton(Constants.ControllerButtonPorts.BUTTON_TWELVE);
    }

    /**
     * Access the x-axis of the left joystick.
     * 
     * @return Left is negative, right is positive.
     */
    public double getLeftJoystickX() {
        return -HARDWARE.getRawAxis(0);
        // This value is negated because something is weird with the wiring.
    }

    /**
     * Access the y-axis of the left joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getLeftJoystickY() {
        return HARDWARE.getRawAxis(1);
    }

    /**
     * Access the x-axis of the right joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getRightJoystickX() {
        return HARDWARE.getRawAxis(2);
    }

    /**
     * Access the y-axis of the right joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getRightJoystickY() {
        return HARDWARE.getRawAxis(3);
    }

    /**
     * A custom button class for the operator board.
     */
    public class OperatorBoardButton extends Button {
        private int m_buttonNumber;

        /**
         * Constructs an OperatorBoardButton with a designated numerical ID
         * 
         * @param button The button's number - a sort of ID
         */
        public OperatorBoardButton(int button) {
            m_buttonNumber = button;
        }

        /**
         * Returns whether the button is being pressed.
         * 
         * @return Whether or not the button is being pressed.
         */
        @Override
        public boolean get() {
            return HARDWARE.getRawButton(m_buttonNumber);
        }

        /**
         * Turns the button's light on, and keeps it on until turnLightOff() is called.
         */
        public void turnLightOn() {
            HARDWARE.setOutput(m_buttonNumber, true);
        }

        /**
         * Turns the button's light off, and keeps it off until turnLightOn() is called.
         */
        public void turnLightOff() {
            HARDWARE.setOutput(m_buttonNumber, false);
        }
    }
}
