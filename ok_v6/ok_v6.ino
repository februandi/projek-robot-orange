#include <AccelStepper.h>
#include <Servo.h>

// Konfigurasi Pin
const int stepPinX = 2;
const int dirPinX = 5;
const int stepPinY = 3;
const int dirPinY = 6;
const int stepPinZ = 4;
const int dirPinZ = 7;
const int stepPinE = 12;
const int dirPinE = 13;

const int enablePin = 8;

const int sensorHomeX = 9;
const int sensorHomeY = 10;
const int sensorHomeZ = 11;
const int sensorHomeE = 16;

const int gripperPin = 17;

// Variabel kontrol servo gripper
int gripperInitialPosition = 0;
int gripperMaxPosition = 70;

// Variabel kontrol motor
int maxSpeed = 5000;
int acceleration = 5000;
int speedHoming = 400;
int awayFromHomeDistanceX = 600;
int awayFromHomeDistanceY = 600;
int awayFromHomeDistanceZ = 1800;
int awayFromHomeDistanceE = 50;
int speedAwayFromHomeDistance = 400;

// Tambahkan batasan untuk setiap sumbu
const int minX = -1000;
const int maxX = 1000;
const int minY = -1000;
const int maxY = 1000;
const int minZ = -1000;
const int maxZ = 1000;
const int minE = -9000;
const int maxE = 0;

// Membuat objek AccelStepper untuk setiap motor
AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, dirPinY);
AccelStepper stepperZ(AccelStepper::DRIVER, stepPinZ, dirPinZ);
AccelStepper stepperE(AccelStepper::DRIVER, stepPinE, dirPinE);

// Membuat objek Servo untuk gripper
Servo gripper;

// Deklarasi fungsi
void homeMotor(AccelStepper &stepper, int sensorPin, bool directionTowardsHome, int awayDistance, int speedHome, int speedAway);
bool handleGcode();
bool processMessage(const String &msg);
int parseValue(const String &command, char axis, int defaultValue);
void sendOk();
void sendError(const String &errorMessage);

void setup() {
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);  // Nonaktifkan semua motor

  pinMode(sensorHomeX, INPUT_PULLUP);
  pinMode(sensorHomeY, INPUT_PULLUP);
  pinMode(sensorHomeZ, INPUT_PULLUP);
  pinMode(sensorHomeE, INPUT_PULLUP);

  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setAcceleration(acceleration);
  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setAcceleration(acceleration);
  stepperZ.setMaxSpeed(maxSpeed);
  stepperZ.setAcceleration(acceleration);
  stepperE.setMaxSpeed(maxSpeed);
  stepperE.setAcceleration(acceleration);

  gripper.attach(gripperPin);
  gripper.write(gripperInitialPosition);

  Serial.begin(115200);
  Serial.println("Robot online");
}

void loop() {
  handleGcode();
}

// Fungsi untuk homing motor
void homeMotor(AccelStepper &stepper, int sensorPin, bool directionTowardsHome, int awayDistance, int speedHome, int speedAway) {
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(speedHome);
  stepper.setAcceleration(acceleration);

  stepper.setSpeed(directionTowardsHome ? -speedHome : speedHome);

  while (digitalRead(sensorPin) == HIGH) {
    stepper.runSpeed();
  }

  stepper.stop();
  stepper.setCurrentPosition(0);

  stepper.setMaxSpeed(speedAway);
  stepper.moveTo(awayDistance * (directionTowardsHome ? 1 : -1));
  while (stepper.isRunning()) {
    stepper.run();
  }

  stepper.setCurrentPosition(0);
}

// Fungsi untuk menangani G-code
bool handleGcode() {
  static String message = "";
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      return false;
    }
    if (c == '\r') {
      // Penanganan overflow buffer
      if (message.length() > 100) { // Misal buffer maksimal 100 karakter
        sendError("Buffer overflow: message too long.");
        message = "";
        return false;
      }

      bool b = processMessage(message);
      message = "";
      return b;
    } else {
      message += c;
    }
  }
  return false;
}

bool processMessage(const String &msg) {
  String command = msg;
  command.toUpperCase();  // Mengubah semua huruf menjadi huruf besar

  if (command.startsWith("G0")) {
    // Penanganan perintah G0
    int targetPosX = parseValue(command, 'X', stepperX.currentPosition());
    int targetPosY = parseValue(command, 'Y', stepperY.currentPosition());
    int targetPosZ = parseValue(command, 'Z', stepperZ.currentPosition());
    int targetPosE = parseValue(command, 'E', stepperE.currentPosition());
    int speed = parseValue(command, 'F', maxSpeed); // Default speed if 'F' is not provided

    // Cek apakah speed kurang dari 50, jika ya, gunakan nilai maxSpeed
    if (speed < 50) {
      speed = maxSpeed;
    }

    // Validasi batasan nilai X, Y, Z, dan E
    if (targetPosX < minX || targetPosX > maxX) {
      sendError("Invalid X position. Allowed range: -1000 to 1000.");
      return false;
    }
    if (targetPosY < minY || targetPosY > maxY) {
      sendError("Invalid Y position. Allowed range: -1000 to 1000.");
      return false;
    }
    if (targetPosZ < minZ || targetPosZ > maxZ) {
      sendError("Invalid Z position. Allowed range: -1000 to 1000.");
      return false;
    }
    if (targetPosE < minE || targetPosE > maxE) {
      sendError("Invalid E position. Allowed range: -9000 to 0.");
      return false;
    }

    Serial.print("Moving to X: ");
    Serial.print(targetPosX);
    Serial.print(" Y: ");
    Serial.print(targetPosY);
    Serial.print(" Z: ");
    Serial.print(targetPosZ);
    Serial.print(" E: ");
    Serial.print(targetPosE);
    Serial.print(" with speed: ");
    Serial.println(speed);

    // Set speed for all steppers
    stepperX.setMaxSpeed(speed);
    stepperY.setMaxSpeed(speed);
    stepperZ.setMaxSpeed(speed);
    stepperE.setMaxSpeed(speed);

    stepperX.moveTo(targetPosX);
    stepperY.moveTo(targetPosY);
    stepperZ.moveTo(targetPosZ);
    stepperE.moveTo(targetPosE);

    while (stepperX.isRunning() || stepperY.isRunning() || stepperZ.isRunning() || stepperE.isRunning()) {
      stepperX.run();
      stepperY.run();
      stepperZ.run();
      stepperE.run();
    }
    sendOk();
    return true;
  } else if (command.startsWith("S")) {
    // Penanganan perintah S (set speed) tidak diperlukan lagi
    // karena speed sudah diatur dalam G0 dengan parameter F
    sendError("Invalid command. Use F parameter in G0 command for speed.");
    return false;
  } else if (command.startsWith("G28")) {
    // Penanganan perintah G28 (homing)
    digitalWrite(enablePin, LOW);
    Serial.println("Homing....");
    homeMotor(stepperX, sensorHomeX, false, awayFromHomeDistanceX, speedHoming, speedAwayFromHomeDistance);
    homeMotor(stepperY, sensorHomeY, true, awayFromHomeDistanceY, speedHoming, speedAwayFromHomeDistance);
    homeMotor(stepperZ, sensorHomeZ, true, awayFromHomeDistanceZ, speedHoming, speedAwayFromHomeDistance);
    homeMotor(stepperE, sensorHomeE, false, awayFromHomeDistanceE, speedHoming, speedAwayFromHomeDistance);

    Serial.println("Homing complete");
    sendOk();
    return true;
  } else if (command.startsWith("T")) {
    // Penanganan perintah T (gripper)
    int angle = command.substring(1).toInt();
    if (angle >= gripperInitialPosition && angle <= gripperMaxPosition) {
      gripper.write(angle);
      Serial.print("Gripper moved to angle: ");
      Serial.println(angle);
      sendOk();
    } else {
      sendError("Invalid gripper angle. Please use a value between the initial and max positions.");
    }
    return true;
  } else if (command.startsWith("M0")) {
    // Penanganan perintah M0 (enable motors)
    digitalWrite(enablePin, LOW);
    Serial.println("Stepper motors ON.");
    sendOk();
    return true;
  } else if (command.startsWith("M1")) {
    // Penanganan perintah M1 (disable motors)
    digitalWrite(enablePin, HIGH);
    Serial.println("Stepper motors OFF.");
    sendOk();
    return true;
  } else if (command.startsWith("G4")) {
    // Penanganan perintah G4 (delay)
    int delayTime = parseValue(command, 'S', 0);
    if (delayTime >= 0) {
      Serial.print("Delaying for ");
      Serial.print(delayTime);
      Serial.println(" milliseconds.");
      delay(delayTime);
      sendOk();
    } else {
      sendError("Invalid delay time.");
    }
    return true;
  } else {
    sendError("Unknown command.");
    return false;
  }
}

int parseValue(const String &command, char axis, int defaultValue) {
  int index = command.indexOf(axis);
  if (index != -1) {
    int endIndex = command.indexOf(' ', index + 1); // Temukan akhir parameter
    if (endIndex == -1) { // Jika tidak ada spasi setelahnya, ambil hingga akhir string
      endIndex = command.length();
    }
    String valueStr = command.substring(index + 1, endIndex); // Ambil nilai antara karakter
    return valueStr.toInt(); // Ubah nilai ke integer
  }
  return defaultValue;
}

void sendOk() {
  Serial.println("ok");
}

void sendError(const String &errorMessage) {
  Serial.print("error: ");
  Serial.println(errorMessage);
}
