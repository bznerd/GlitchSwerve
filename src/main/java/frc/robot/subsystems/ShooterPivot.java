package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kShooter.kPivot;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;

public class ShooterPivot extends SubsystemBase {
  // Motorcontrollers
  private final CANSparkMax pivotMotor1;
  private final CANSparkMax pivotMotor2;

  // Feedforwards
  private final ArmFeedforward pivotFF;

  // PID controllers
  private final SparkPIDController pivotPID;

  // Absolute Encoders
  private final AbsoluteEncoder pivotEncoder;

  // Profile Stuff
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.kProfile.maxVel, kPivot.kProfile.maxAccel);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ShooterPivot() {
    // Motor Initializations
    pivotMotor1 =
        getSparkMax(
            kPivot.pivot1MotorID,
            CANSparkLowLevel.MotorType.kBrushless,
            true,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    pivotMotor2 =
        getFollower(pivotMotor1, kPivot.pivot2MotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
    pivotMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

    // Feed Forwards
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    // Encoder Configs
    pivotEncoder = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivotEncoder.setInverted(false);

    // PID Configs
    pivotPID = pivotMotor1.getPIDController();
    pivotPID.setFeedbackDevice(pivotEncoder);
    pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivotPID.setPositionPIDWrappingEnabled(false);
    pivotPID.setP(kPivot.kP);
    pivotPID.setD(kPivot.kD);
  }

  private TrapezoidProfile.State calculateSetpoint(double posRad) {
    goal = new TrapezoidProfile.State(posRad, 0);
    setpoint = profile.calculate(kPivot.period, setpoint, goal);
    return setpoint;
  }

  private boolean getDone() {
    return goal.position == pivotEncoder.getPosition()
        && goal.velocity == pivotEncoder.getVelocity();
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
            () -> {
              TrapezoidProfile.State pos = calculateSetpoint(posRad);
              pivotPID.setReference(
                  pos.position,
                  ControlType.kPosition,
                  0,
                  pivotFF.calculate(pos.position, pos.velocity));
            })
        .until(this::getDone);
  }

  // Get SysID Routine
  public SysIdRoutine getAngularRoutine() {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
    MutableMeasure<Angle> angle = mutable(Radians.of(0));
    // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
    MutableMeasure<Velocity<Angle>> velocity = mutable(RadiansPerSecond.of(0));
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (volts) -> {
              pivotMotor1.setVoltage(volts.magnitude());
            },
            (log) -> {
              pivotEncoder.setVelocityConversionFactor(Math.PI);
              log.motor("shooterPivotMotor")
                  .voltage(appliedVoltage.mut_replace(pivotMotor1.getBusVoltage(), Volts))
                  .angularPosition(angle.mut_replace(pivotEncoder.getPosition() * Math.PI, Radians))
                  .angularVelocity(
                      velocity.mut_replace(
                          (pivotEncoder.getVelocity() * Math.PI), RadiansPerSecond));
            },
            this));
  }
}
