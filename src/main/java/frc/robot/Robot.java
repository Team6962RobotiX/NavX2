// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive straight.
 * This program uses a joystick to drive forwards and backwards while the gyro is used for direction
 * keeping.
 */
public class Robot extends TimedRobot {
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant

  private static final int kLeftMotorPort = 0;
  private static final int kRightMotorPort = 1;

  private static final int kJoystickPort = 0;

  private final CANSparkMax m_leftDrive = new CANSparkMax(kLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive = new CANSparkMax(kRightMotorPort, MotorType.kBrushless);
  private final DifferentialDrive m_myRobot = new DifferentialDrive(m_leftDrive, m_rightDrive);
  // private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);


  @Override
  public void robotInit() {
    m_gyro.calibrate();
    m_gyro.getPitch();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /**
   * The motor speed is set from the joystick while the DifferentialDrive turning value is assigned
   * from the error between the setpoint and the gyro angle.
   */
  @Override
  public void teleopPeriodic() {
    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
    m_myRobot.arcadeDrive(-m_joystick.getY(), -turningValue);
  }
}
