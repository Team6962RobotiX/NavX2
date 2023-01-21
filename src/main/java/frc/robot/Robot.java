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

import java.util.ArrayList;
import java.util.Random;

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

  private final CANSparkMax m_leftDrive = new CANSparkMax(kLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive = new CANSparkMax(kRightMotorPort, MotorType.kBrushless);
  private final DifferentialDrive m_myRobot = new DifferentialDrive(m_leftDrive, m_rightDrive);
  // private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

  private final long initTime = System.currentTimeMillis();


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
    boolean st1 = false;
    boolean st2 = false;
    double angleRatio = m_gyro.getPitch()/180;

    
    while (!st1) {
      m_myRobot.tankDrive(1.0, 1.0);
      if (m_gyro.getPitch() > 25) {
        st1 = true;
      }
    } while (!st2) {
      m_myRobot.tankDrive(angleRatio, angleRatio);
      st2 = true;
    }
  }

  public void teleopPeriodic() {

    long currentTime = initTime - System.currentTimeMillis();

    
    boolean st1 = false;
    boolean st2 = false;
    double angleRatio = m_gyro.getPitch()/180;

    
    while (!st1) {
      m_myRobot.tankDrive(1.0, 1.0);
      if (getLyse((int)(currentTime/100)) > 25) {
        st1 = true;
      }
    } while (!st2) {
      m_myRobot.tankDrive(angleRatio, angleRatio);
      st2 = true;
    }
  }





  private static Random rand = new Random();   
  public static Float getLyse(int time) {
    ArrayList<Float> values = new ArrayList<Float>();
    for(int i = 0; i < 100; i++){
      values.add(randValue());
    }
		return values.get(time);
  }

  public static float randValue(){
    return rand.nextFloat()*68 - 34;
  }

}