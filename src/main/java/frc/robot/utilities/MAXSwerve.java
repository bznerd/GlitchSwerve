package frc.robot.utilities;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants.kSwerve.kModule;

public class MAXSwerve {
  private SwerveModuleState targetState = new SwerveModuleState();
  private final double chassisOffset;

  // Hardware
  private final CANSparkMax driveNEO;
  private final CANSparkMax steerNEO;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  // Controls
  private final SparkMaxPIDController drivePID;
  private final SparkMaxPIDController steerPID;
  private final SimpleMotorFeedforward driveFF;

  // Logging
  private final NetworkTable moduleTable;
  private final DoublePublisher goalVelPub;
  private final DoublePublisher goalHeadingPub;
  private final DoublePublisher measVelPub;
  private final DoublePublisher measHeadingPub;
  private final DoubleArrayPublisher voltagesPub;

  public MAXSwerve(int driveCANId, int steerCANId, double offset, NetworkTable table) {
    chassisOffset = offset;

    // Initialize hardware
    driveNEO = new CANSparkMax(driveCANId, MotorType.kBrushless);
    steerNEO = new CANSparkMax(steerCANId, MotorType.kBrushless);

    driveEncoder = driveNEO.getEncoder();
    steerEncoder = steerNEO.getAbsoluteEncoder(Type.kDutyCycle);

    driveEncoder.setPositionConversionFactor(kModule.drivingEncoderPositionFactor);
    steerEncoder.setPositionConversionFactor(kModule.steeringEncoderPositionFactor);

    driveEncoder.setVelocityConversionFactor(kModule.drivingEncoderVelocityFactor);
    steerEncoder.setVelocityConversionFactor(kModule.steeringEncoderVelocityFactor);

    steerEncoder.setInverted(kModule.steeringEncoderInverted);

    // Initialize controls objects
    drivePID = driveNEO.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    steerPID = steerNEO.getPIDController();
    steerPID.setFeedbackDevice(steerEncoder);

    driveFF = new SimpleMotorFeedforward(kModule.drivingS, kModule.drivingV, kModule.drivingA);

    drivePID.setOutputRange(kModule.drivingMinOutput, kModule.drivingMaxOutput);
    steerPID.setOutputRange(kModule.steeringMinOutput, kModule.steeringMaxOutput);

    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMaxInput(kModule.steeringEncoderPositionPIDMaxInput);
    steerPID.setPositionPIDWrappingMinInput(kModule.steeringEncoderPositionPIDMinInput);

    drivePID.setP(kModule.drivingP);
    drivePID.setI(kModule.drivingI);
    drivePID.setD(kModule.drivingD);

    steerPID.setP(kModule.drivingP);
    steerPID.setI(kModule.drivingI);
    steerPID.setD(kModule.drivingD);

    // Configure motor controllers
    driveNEO.setIdleMode(IdleMode.kBrake);
    steerNEO.setIdleMode(IdleMode.kBrake);

    driveNEO.setSmartCurrentLimit(kModule.drivingMotorCurrentLimit);
    steerNEO.setSmartCurrentLimit(kModule.steeringMotorCurrentLimit);

    driveNEO.burnFlash();
    steerNEO.burnFlash();

    targetState.angle = new Rotation2d(steerEncoder.getPosition());

    // Start NT publishers for logging
    moduleTable = table;
    goalVelPub = moduleTable.getDoubleTopic("Goal Velocity").publish();
    goalHeadingPub = moduleTable.getDoubleTopic("Goal Heading").publish();
    measVelPub = moduleTable.getDoubleTopic("Measured Velocity").publish();
    measHeadingPub = moduleTable.getDoubleTopic("Measured Heading").publish();
    voltagesPub = moduleTable.getDoubleArrayTopic("Voltages").publish();
  }

  // Get the corrected (for chassis offset) heading
  public Rotation2d getCorrectedSteer() {
    return new Rotation2d(steerEncoder.getPosition() + chassisOffset);
  }

  // Get the state of the module (vel, heading)
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getCorrectedSteer());
  }

  // Get the position of the module (wheel distance traveled, heading)
  public SwerveModulePosition getPositon() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getCorrectedSteer());
  }

  // Set the module's target state
  public void setTargetState(SwerveModuleState desiredState, boolean closedLoopDrive) {
    // Optimize the state to prevent having to make a rotation of more than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(
            new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.minus(new Rotation2d(chassisOffset))),
            new Rotation2d(steerEncoder.getPosition()));

    // Set the built-in PID for closed loop, or just give a regular voltage for open loop
    if (closedLoopDrive) {
      drivePID.setReference(
          optimizedState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          driveFF.calculate(optimizedState.speedMetersPerSecond));
    } else {
      driveNEO.setVoltage(driveFF.calculate(optimizedState.speedMetersPerSecond));
    }

    steerPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

    // Record the target state
    targetState = optimizedState;
  }

  // Set the module to the chassis X configuraiton
  public void setX() {
    setTargetState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + chassisOffset)), false);
  }

  // Reset the drive encoder to zero (reset for odometry)
  public void resetEncoder() {
    driveEncoder.setPosition(0);
  }

  // Put the drive motors into or out of brake mode
  public void setBrakeMode(boolean brake) {
    if (brake) {
      driveNEO.setIdleMode(IdleMode.kBrake);
    } else {
      driveNEO.setIdleMode(IdleMode.kCoast);
    }
  }

  // Update the logging values in NT
  public void updateNT() {
    goalVelPub.set(targetState.speedMetersPerSecond);
    goalHeadingPub.set(targetState.angle.getRadians());
    measVelPub.set(driveEncoder.getVelocity());
    measHeadingPub.set(steerEncoder.getPosition());
    voltagesPub.set(
        new double[] {
          driveNEO.getAppliedOutput() * driveNEO.getBusVoltage(),
          steerNEO.getAppliedOutput() * steerNEO.getBusVoltage()
        });
  }
}
