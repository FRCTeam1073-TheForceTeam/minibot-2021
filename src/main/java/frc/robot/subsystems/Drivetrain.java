// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Drivetrain extends SubsystemBase {
  // Drive System Constants
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.07;  // 70mm
  private static final double kVelocity = 1.0; // TODO: Improve, this is current estimate. 0.4m/s / 0.5 PWM
  private static final double kTorque = 1.0;    // Convert PID output into PWM scale.
  private static final double kP = 1.0;         // TODO: Tune PID velocity loop.
  private static final double kI = 0.07;
  private static final double kD = 0.0;


  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);
 // private AnalogInput voltageInput = new AnalogInput(2);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Encoder glitch tracking:
  private int m_leftEncoderGlitches = 0;
  private int m_rightEncoderGlitches = 0;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  //controller state variables
  private LinearFilter m_leftVelocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02); //(time constant, period)
  private LinearFilter m_rightVelocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
  private SlewRateLimiter m_leftSlew = new SlewRateLimiter(0.1);
  private SlewRateLimiter m_rightSlew = new SlewRateLimiter(0.1);
  private double m_lCommandedV = 0;
  private double m_rCommandedV = 0;
  private double m_leftVelocity = 0;
  private double m_rightVelocity = 0;
  private SimpleMotorFeedforward m_leftMotorFF = new SimpleMotorFeedforward(0.0, kVelocity, kTorque);
  private SimpleMotorFeedforward m_rightMotorFF = new SimpleMotorFeedforward(0.0, kVelocity, kTorque); 
  private PIDController m_leftPID = new PIDController(kP, kI, kD);
  private PIDController m_rightPID = new PIDController(kP, kI, kD);
  private Boolean m_closedLoopControl = false;


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances:
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);

    // Limit integrator windup:
    m_leftPID.setIntegratorRange(-10, 10);
    m_rightPID.setIntegratorRange(-10, 10);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_closedLoopControl = false; // Disable closed loop control.
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /* Set left and right velocities in m/s */
  public void velocityDrive(double leftV, double rightV) {
    // Save our commanded velocity:
    m_lCommandedV = leftV;
    m_rCommandedV = rightV;

    // If we were not in closed loop control, then reset PIDs and switch to closed loop control.
    if (!m_closedLoopControl) {
      m_leftPID.reset();
      m_rightPID.reset();
    }
    m_closedLoopControl = true;
  }

  // Set the PWMs of each motor driectly... open loop control
  public void setPWMs(double leftPWM, double rightPWM) {
    // Turn off closed loop control.
    m_closedLoopControl = false;
    // Directly set PWMs /w clamped inputs.
    m_leftMotor.setSpeed(MathUtil.clamp(leftPWM, -1.0, 1.0));
    m_rightMotor.setSpeed(MathUtil.clamp(rightPWM, -1.0, 1.0));
  }

  // Report current wheel speeds.
  public void getWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    speeds.leftMetersPerSecond = m_leftVelocity;
    speeds.rightMetersPerSecond = m_rightVelocity;
  }

  @Override
  public void periodic() {
    m_diffDrive.feed();

    // This method will be called once per scheduler run
    double leftEncoder = m_leftEncoder.getDistance();
    double rightEncoder = m_rightEncoder.getDistance();
    SmartDashboard.putNumber("leftEncoder", leftEncoder);
    SmartDashboard.putNumber("rightEncoder", rightEncoder);

    //double voltage = voltageInput.getVoltage();
    //SmartDashboard.putNumber("voltage", voltage);

    // // Anti-Glitch filter
    // if (m_leftVelocity * m_leftEncoder.getRate() < 0) {
    //   // leftEncoder = m_lastLeftEncoder;
    //   m_leftEncoderGlitches++;
    //   System.out.println("Left Glitch: " );
    // }

    // if (m_rightVelocity * m_rightEncoder.getRate() < 0) {
    //   // rightEncoder = m_lastRightEncoder;
    //   m_rightEncoderGlitches++;
    //   System.out.println("Right Glitch: ");
    // }

    m_leftVelocity = m_leftVelocityFilter.calculate(m_leftEncoder.getRate());
    m_rightVelocity = m_rightVelocityFilter.calculate(m_rightEncoder.getRate());

    SmartDashboard.putNumber("leftVelocity", m_leftVelocity);
    SmartDashboard.putNumber("rightVelocity", m_rightVelocity);

    // TODO: Odometry update...later

    // If we have a closed loop command run our closed loop controller.
    if (m_closedLoopControl) {
      // run PID controller, returns output of trajectory function
      // compute refV using a slew rate limiter
      double leftRef = m_leftSlew.calculate(m_lCommandedV);
      double rightRef = m_rightSlew.calculate(m_rCommandedV);

      SmartDashboard.putNumber("leftRef", leftRef);
      SmartDashboard.putNumber("rightRef", rightRef);

      // Run PID controllers to track this velocity.
      double leftTorque = m_leftPID.calculate(m_leftVelocity, leftRef);
      double rightTorque = m_rightPID.calculate(m_rightVelocity, rightRef);

      // Clamp the torques to the max we will allow.
      leftTorque = MathUtil.clamp(leftTorque, -10.0, 10.0);
      rightTorque = MathUtil.clamp(rightTorque, -10.0, 10.0);

      // Temp Testing:
      // leftTorque = 0;
      // rightTorque = 0;

      SmartDashboard.putNumber("leftTorque", leftTorque);
      SmartDashboard.putNumber("rightTorque", rightTorque);

      // Calc the ff for motor(s):
      // Velocity feed forward is based on motor velocity feedback.
      // Acceleration feed forward is based on controller torque outputs.
      double leftOutput = m_leftMotorFF.calculate(m_leftVelocity, leftTorque);
      double rightOutput = m_rightMotorFF.calculate(m_rightVelocity, rightTorque);

      // Set motor pwm (making it move) with output clamping:
      double leftPWM = MathUtil.clamp(leftOutput, -1.0, 1.0);
      double rightPWM = MathUtil.clamp(rightOutput, -1.0, 1.0);

      SmartDashboard.putNumber("leftPWM", leftPWM);
      SmartDashboard.putNumber("rightPWM", rightPWM);  

      m_leftMotor.setSpeed(leftPWM);
      m_rightMotor.setSpeed(-rightPWM); // Sign of movement is opposite encoder for the right motor.
      // TODO: Investigate what setVoltage() really does.
      // m_leftMotor.setVoltage(leftPWM);
      // m_rightMotor.setVoltage(-rightPWM);
    }
  }


}
