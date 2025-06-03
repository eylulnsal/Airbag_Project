import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050

# Definition der Pins für den Ultraschallsensor und das Servo
TRIG = 23
ECHO = 24
SERVO_PIN = 18

# Initialisierung des MPU6050 Beschleunigungssensors
sensor = mpu6050(0x68)

# GPIO-Modus festlegen
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialisierung des Servos mit 50 Hz PWM-Frequenz
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Kalman-Filter Klasse zur Glättung der Beschleunigungsdaten
class KalmanFilter:
    def __init__(self, Q=0.01, R=0.1):
        self.Q = Q  # Prozessrauschen
        self.R = R  # Messrauschen
        self.x = 0  # Schätzung
        self.P = 1  # Fehlerkovarianz

    # Update-Funktion des Kalman-Filters
    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

# Instanz des Kalman-Filters
kf = KalmanFilter()

# Funktion zur Entfernungsmessung mit Ultraschallsensor
def distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout_start = time.time()
    pulse_start = None
    pulse_end = None

    # Warten bis das Echo HIGH ist (Startzeit)
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.04:
            return 999  # Zeitüberschreitung

    timeout_start = time.time()

    # Warten bis das Echo LOW ist (Endzeit)
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.04:
            return 999  # Zeitüberschreitung

    if pulse_start is None or pulse_end is None:
        return 999  # Messung fehlgeschlagen

    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 17150  # Umrechnung in cm
    return round(distance_cm, 2)

# Funktion zur Messung der Beschleunigung und Anwendung des Kalman-Filters
def read_acceleration():
    accel_data = sensor.get_accel_data()
    ax = abs(accel_data['x'])
    ay = abs(accel_data['y'])
    az = abs(accel_data['z'])
    total_acc = (ax**2 + ay**2 + az**2)**0.5  # Gesamtbeschleunigung berechnen
    filtered_acc = kf.update(total_acc)  # Gefilterter Wert
    return filtered_acc

# Funktion zum Auslösen und Zurücksetzen des Airbags (Servo-Steuerung)
def open_airbag():
    print("Airbag wird ausgelöst!")
    servo.ChangeDutyCycle(14)  # Servo in Auslöseposition
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

    print("Wartezeit: 5 Sekunden...")
    time.sleep(5)

    print("Airbag wird zurückgesetzt.")
    servo.ChangeDutyCycle(2)  # Servo in Ausgangsposition
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

    print("System wird heruntergefahren.")
    GPIO.cleanup()
    exit()

# Hauptprogrammschleife
try:
    while True:
        dist = distance()
        acc = read_acceleration()

        print(f"Entfernung: {dist} cm | Gefilterte Beschleunigung: {acc:.2f} m/s²")

        # Auslösungskriterium: nahe Objekt und hohe Beschleunigung
        if dist < 5 and acc > 12.0:
            open_airbag()

        time.sleep(0.1)

# Falls das Programm mit Strg+C beendet wird
except KeyboardInterrupt:
    print("Vom Benutzer unterbrochen.")
    GPIO.cleanup()
