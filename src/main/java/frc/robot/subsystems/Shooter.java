package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

/**
 * =========================================================================================
 * SUBSISTEMA: SHOOTER (LANZADOR DE PIEZAS / NOTAS)
 * =========================================================================================
 * ¿Para qué sirve?
 *   Lanza las notas hacia el Speaker o Amp con velocidad regulada.
 *   Utiliza un sistema de dos volantes (Flywheels) independientes que permiten aplicar efecto
 *   (Spin) a la nota y ajustar la potencia automáticamente según la distancia calculada por la Limelight.
 *
 * Hardware asociado:
 *   - Motor Izquierdo: TalonFX / Kraken X60 (CTRE Phoenix 6) en CAN ID 23 (en el bus CANivore del Swerve).
 *   - Motor Derecho: SparkMax / NEO (REV Robotics) en CAN ID 16.
 * =========================================================================================
 */
public class Shooter extends SubsystemBase {

    // --- IDENTIFICADORES DE BUS CAN ---
    public static final int MOTOR_LEFT_CAN_ID = 23;     // TalonFX (Kraken X60)
    public static final int MOTOR_RIGHT_CAN_ID = 16;    // SparkMax (NEO)

    // --- VELOCIDADES BASE DE DISPARO ---
    public static final double SHOOT_SPEED = 0.5;              // 50% potencia para disparo estándar
    public static final double SHOOT_SPEED_RIGTH = -1.0;       // Potencia inversa completa en motor derecho
    public static final double SHOOT_FULL_SPEED = 1.0;         // 100% de potencia

    // --- TIEMPOS Y PARÁMETROS DE DISPARO AUTOMÁTICO ---
    private static final double SPIN_UP_DELAY = 0.4;           // Tiempo de aceleración antes de alimentar nota (400ms)
    private static final double DISTANCIA_DEFAULT_AUTO = 2.5;  // Distancia asumida si la cámara no ve AprilTag en Autónomo (2.5m)

    // --- MODELO MATEMÁTICO: VELOCIDAD EN FUNCIÓN DE LA DISTANCIA ---
    // Ecuación lineal: Potencia = PotenciaMin + (Distancia - DistanciaMin) * Pendiente
    private static final double DISTANCIA_MIN_METROS = 1.0;    // Distancia mínima efectiva
    private static final double POTENCIA_MINima = 0.4;          // Potencia base al estar pegado al objetivo (40%)
    private static final double POTENCIA_MAXima = 1.0;          // Potencia tope (100%)
    private static final double PENDIENTE_POTENCIA = 0.15;     // Incremento de potencia por cada metro extra

    // --- HARDWARE Y CONTROLADORES ---
    /** Motor derecho controlado por SparkMax (REV) */
    private final SparkMax motorRight = new SparkMax(MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
    /** Temporizador para sincronizar la aceleración de los volantes */
    private final Timer spinUpTimer = new Timer();

    /** Motor izquierdo de alta potencia controlado por Phoenix 6 (CTRE) */
    private final TalonFX motorLeft;
    private final TalonFXConfiguration leftConfig;
    private final DutyCycleOut m_leftRequest;

    /**
     * Constructor del subsistema Shooter:
     * - Configura ambos motores en modo Coast (libre) para permitir giro rápido y eficiente.
     * - Configura límites de corriente del estator en 60A para proteger los bobinados.
     */
    public Shooter() {
        // 1. Configuración del motor derecho SparkMax
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.smartCurrentLimit(60);
        motorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 2. Configuración del motor izquierdo TalonFX (Kraken)
        this.motorLeft = new TalonFX(MOTOR_LEFT_CAN_ID, TunerConstants.kCANBus.getName());
        this.leftConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        this.m_leftRequest = new DutyCycleOut(0.50);
    }

    // =====================================================================================
    // MÉTODOS BÁSICOS DE DISPARO
    // =====================================================================================

    /** Enciende ambos volantes a velocidad fija estándar */
    public void shoot() {
        motorLeft.setControl(m_leftRequest.withOutput(SHOOT_SPEED));
        motorRight.set(SHOOT_SPEED_RIGTH);
    }

    /** Enciende ambos volantes al 100% de potencia */
    public void shootFull() {
        motorLeft.setControl(m_leftRequest.withOutput(SHOOT_FULL_SPEED));
        motorRight.set(-SHOOT_FULL_SPEED);
    }

    /** Apaga ambos motores de inmediato */
    public void stop() {
        motorLeft.setControl(m_leftRequest.withOutput(0));
        motorRight.set(0);
    }

    /**
     * Calcula la potencia requerida según la distancia en metros calculada por Limelight.
     * @param distanciaMetros Distancia horizontal al AprilTag
     */
    public void dispararSegunDistancia(double distanciaMetros) {
        if (distanciaMetros <= 0) {
            stop();
            return;
        }

        // Interpolación lineal de velocidad
        double velocidadIzq = POTENCIA_MINima + (distanciaMetros - DISTANCIA_MIN_METROS) * PENDIENTE_POTENCIA + 0.02;
        velocidadIzq = Math.min(Math.max(velocidadIzq, POTENCIA_MINima), POTENCIA_MAXima);

        // Mantiene la relación de efecto de giro entre rodillo derecho e izquierdo
        double ratioSpin = SHOOT_SPEED_RIGTH / SHOOT_SPEED;
        double velocidadDer = velocidadIzq * ratioSpin;

        motorLeft.setControl(m_leftRequest.withOutput(velocidadIzq));
        motorRight.set(velocidadDer);
    }

    @Override
    public void periodic() {
        // Telemetría periódica si se requiere
    }

    // =====================================================================================
    // FÁBRICAS DE COMANDOS (COMMANDS)
    // =====================================================================================

    /** Dispara a potencia fija con aceleración previa mediante temporizador */
    public Command shootCommand() {
        return Commands.runEnd(
                () -> {
                    motorRight.set(SHOOT_SPEED_RIGTH);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? SHOOT_SPEED : 0.0));
                },
                () -> {
                    stop();
                    spinUpTimer.stop();
                },
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    /** Dispara al 100% de potencia */
    public Command shootFullCommand() {
        return Commands.runEnd(
                () -> {
                    motorRight.set(-SHOOT_FULL_SPEED);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? SHOOT_FULL_SPEED : 0.0));
                },
                () -> {
                    stop();
                    spinUpTimer.stop();
                },
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    /**
     * Comando inteligente que lee en tiempo real la distancia de la Limelight
     * y modula la velocidad de ambos motores continuamente.
     */
    public Command dispararSegunDistanciaCommand(DoubleSupplier distanceSupplier) {
        return Commands.runEnd(
                () -> {
                    double dist = distanceSupplier.getAsDouble();

                    // Si estamos en autonómo y no hay objetivo visible, usamos la distancia predeterminada (2.5m)
                    if (dist <= 0 && DriverStation.isAutonomous()) {
                        dist = DISTANCIA_DEFAULT_AUTO;
                    }

                    if (dist <= 0) {
                        stop();
                        return;
                    }

                    double velocidadIzq = POTENCIA_MINima + (dist - DISTANCIA_MIN_METROS) * PENDIENTE_POTENCIA + 0.02;
                    velocidadIzq = Math.min(Math.max(velocidadIzq, POTENCIA_MINima), POTENCIA_MAXima);

                    double ratioSpin = SHOOT_SPEED_RIGTH / SHOOT_SPEED;
                    double velocidadDer = (velocidadIzq * ratioSpin) - 0.05; // Ajuste fino de potencia

                    motorRight.set(velocidadDer);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? velocidadIzq : 0.0));
                },
                this::stop,
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    public Command ShootStop() {
        return Commands.run(this::stop, this);
    }
}
