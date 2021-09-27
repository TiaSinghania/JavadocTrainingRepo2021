/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**

 * Our wrapper for the absolute encoder, such as the US Digital MA3.  
 * Most useful for tracking the position of swerve wheels.

 */
public class AbsoluteEncoder {
    private AnalogInput m_analogIn;
    private boolean m_isInverted;

    private AnalogInputSim m_analogInSim;

    private double VOLTAGE_TO_RADIANS = Math.PI * 2 / 5;

    private double m_offset;

    /**
     * Construct and absolute encoder, most likely a US Digital MA3 encoder.
     * 
     * @param channel  analog in (sometimes also refered to as AIO) port on the
     *                 roboRIO

     * @param inverted set this to <i>TRUE</i> if physically turning the swerve
     *                 wheel <i>CLOCKWISE</i> (looking down on it from the top of
     *                 the bot) <i>INCREASES</i> the raw voltage the encoder
     *                 returns.
     * @param offset   swerve offset (in <i>RADIANS</i>). This value is
     *                 <i>SUBTRACTED</i> from the raw encoder output, so double
     *                 check your algebra!
     */
    public AbsoluteEncoder(int channel, boolean inverted, double offset) {
        m_analogIn = new AnalogInput(channel);
        m_analogInSim = new AnalogInputSim(m_analogIn);
        m_isInverted = inverted;
        m_offset = offset;
    }

    /**
     * 

     * @return the angle of the absolute encoder, as a Rotation2d object. zero
     *         points toward the front of the bot. <i>the value increases as the
     *         swerve wheel is turned counter-clockwise.</i>
     */

    public Rotation2d getRotation2d() {

        if (m_isInverted) {
            return new Rotation2d((5 - m_analogIn.getVoltage()) * VOLTAGE_TO_RADIANS - m_offset);
        }

        return new Rotation2d((m_analogIn.getVoltage()) * VOLTAGE_TO_RADIANS - m_offset);
    }


    /**
     * Figures out the value that the simulated absolute encoder should be at. Read
     * comments in source code below.
     * 
     * @param turnVoltage Voltage that will go to the turning motor, range: [-1, 1].
     */
    public void sendVoltage(double turnVoltage) {
        // gear ratio between motor and wheel / encoder
        double gearRatio = 12.8;

        // maximum RPM for motor under 0 load
        double motorRPM = 5500;

        // how long it takes to run one loop of the periodic (in seconds)
        double tickPeriod = Robot.kDefaultPeriod;

        if (turnVoltage > 1) {
            turnVoltage = 1;
        } else if (turnVoltage < -1) {
            turnVoltage = -1;
        }

        // started with motorRPM and converted to wheel rotations per tick
        // RPM * (min / sec) * (s / tick) * (wheel rotations / motor rotations)
        double wheelRotationsPerTick = motorRPM / 60 * tickPeriod / gearRatio; // 0.143
        double wheelRotationsSinceLastTick = wheelRotationsPerTick * turnVoltage;
        double voltsSinceLastTick = 5 * wheelRotationsSinceLastTick;
        double polarity = m_isInverted ? -1 : 1;
        voltsSinceLastTick *= polarity;

        double outputVoltage = m_analogIn.getVoltage() + voltsSinceLastTick;

        SmartDashboard.putNumber("turning voltage" + m_analogIn.getChannel(), turnVoltage);

        // convert output to a number between 0 and 5 (continuous unit circle)
        outputVoltage = (((outputVoltage % 5) + 5) % 5);

        // basically this "hijacks" the simulated absolute encoder to say that it's
        // reading the voltage that u give it, range: [0, 5]
        m_analogInSim.setVoltage(outputVoltage);
    }

}