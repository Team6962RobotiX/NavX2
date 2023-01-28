// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.networktables.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.lang.Math;

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive straight.
 * This program uses a joystick to drive forwards and backwards while the gyro is used for direction
 * keeping.
 */
public class Robot extends TimedRobot {
  private static final double kP = 0.005; // propotional turning constant

  private static final int kLeftMotorPort = 0;
  private static final int kRightMotorPort = 1;

  private static final int kJoystickPort = 0;

  // private final CANSparkMax m_leftDrive = new CANSparkMax(kLeftMotorPort, MotorType.kBrushless);
  // private final CANSparkMax m_rightDrive = new CANSparkMax(kRightMotorPort, MotorType.kBrushless);
  PWMSparkMax m_leftDrive = new PWMSparkMax(kLeftMotorPort);
  PWMSparkMax m_rightDrive = new PWMSparkMax(kRightMotorPort);
  private final DifferentialDrive m_myRobot = new DifferentialDrive(m_leftDrive, m_rightDrive);
  // private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);
  boolean autoBalanceXMode;
  final double kOffBalanceAngleThresholdDegrees = 10;
  final double kOonBalanceAngleThresholdDegrees = 5;

  // private final long initTime = System.currentTimeMillis();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("gyro");
  final DoublePublisher pitchPub = inst.getDoubleTopic("pitch").publish();
  final DoublePublisher outPub = inst.getDoubleTopic("out").publish();

  @Override
  public void robotInit() {
    m_gyro.calibrate();
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
  public void robotPeriodic() {
    // boolean st1 = false;
    // boolean st2 = false;
    // double angleRatio = m_gyro.getPitch() / 180;

    // while (!st1) {
    //   m_myRobot.tankDrive(1.0, 1.0);
    //   if (m_gyro.getPitch() > 25) {
    //     st1 = true;
    //   }
    // }
    // while (!st2) {
    //   m_myRobot.tankDrive(angleRatio, angleRatio);
    //   st2 = true;
    // }
  }

  public void teleopPeriodic() {
    double xAxisRate = m_joystick.getX();
    double pitchAngleDegrees = m_gyro.getPitch();

    if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    } else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    if (autoBalanceXMode) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }

    try {
      m_myRobot.tankDrive(xAxisRate, xAxisRate);
    } catch (RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
    }

    // timer for motor update time?

    // long currentTime = initTime - System.currentTimeMillis();

    // boolean st1 = false;
    // boolean st2 = false;
    // double angleRatio = m_gyro.getPitch() / 180;

    // pitchPub.set(m_gyro.getPitch());

    // while (!st1) {
    //   m_myRobot.tankDrive(1.0, 1.0);
    //   if (m_gyro.getPitch() > 25) {
    //     st1 = true;
    //     outPub.set(1);
    //   }
    // }
    // while (!st2) {
    //   m_myRobot.tankDrive(angleRatio, angleRatio);
    //   st2 = true;
    //   outPub.set(angleRatio);
    // }
  }
}