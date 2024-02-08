package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
    
    private final RelativeEncoder encoder = motor1.getEncoder();

    private final Constraints constraints = new Constraints(1.0, 0.5);
    private final ProfiledPIDController controller = new ProfiledPIDController(0.1, 0.01, 0.001, constraints);

    public ArmSubsystem() {
        // Configurações adicionais podem ser necessárias, como inversões de motor
        motor2.follow(motor1, true);
    }

    public void setArmPosition(double setpoint) {
        double currentPosition = encoder.getPosition();
        double output = controller.calculate(currentPosition, setpoint);
        motor1.set(output);
    }

    public void stopArm() {
        motor1.stopMotor();
    }

    @Override
    public void periodic() {
        // Atualizações periódicas, se necessário
    }
}