//A go to position A (A-F)
//80,120,125,50,10 Move to positon
//A,C Go to position A to C (A-F)
//X Go to Top
//Y Close gripper, stay in position
//Z Open gripper, stay in position

#include <Braccio++.h>
#define SERVO_PIN 6
#define TIME_DELAY 2000  // general delay between poses

// ------------------ Predefined positions ------------------
float POS_A[6] = { 0, 134, 95, 95, 140, 0 };
float POS_B[6] = { 0, 134, 70, 150, 110, 50 };
float POS_C[6] = { 0, 134, 90, 95, 145, 100 };
float POS_D[6] = { 0, 134, 70, 150, 110, 150 };
float POS_E[6] = { 0, 134, 95, 95, 140, 200 };
float POS_F[6] = { 0, 134, 70, 135, 120, 250 };

// Top positions (safe travel heights)
//{ 0, 100, 157, 157, 157, 0 }
const float FIXED_TOP[6] = { 0, 100, 157, 157, 157, 0 };
float TOP[6] = { 0, 100, 140, 140, 140, 0 };  // claw value filled dynamically

// ------------------ Globals ------------------
float currentPose[6] = { 0, 134, 100, 95, 140, 60 };
int clawAngle = 10;  // start "open"

// ------------------ Helpers ------------------
void writeClawPWM(int angle) {
  int pwm = map(angle, 0, 180, 544, 2400);
  analogWrite(SERVO_PIN, pwm / 4);
  clawAngle = angle;
}

void moveArm(float pose[6], int claw = -1) {
  Braccio.moveTo(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  delay(TIME_DELAY);
  if (claw >= 0) writeClawPWM(claw);
  Serial.println("Done");
}

float* getPose(char id) {
  switch (toupper(id)) {
    case 'A': return POS_A;
    case 'B': return POS_B;
    case 'C': return POS_C;
    case 'D': return POS_D;
    case 'E': return POS_E;
    case 'F': return POS_F;
  }
  return nullptr;
}

// Convert letter A-F to numeric index used for pickAndPlace
int letterToIndex(char c) {
  switch (toupper(c)) {
    case 'A': return 1;
    case 'B': return 2;
    case 'C': return 3;
    case 'D': return 4;
    case 'E': return 5;
    case 'F': return 6;
    default: return -1;
  }
}

// ------------------ Pick & Place ------------------
void moveObject(char src, char dst) {
  float* pSrc = getPose(src);
  float* pDst = getPose(dst);
  if (!pSrc || !pDst) return;

  // Determine delay for top-to-top moves based on distance
  int srcIdx = letterToIndex(src);
  int dstIdx = letterToIndex(dst);
  int distance = abs(dstIdx - srcIdx);
  int topMoveDelay = TIME_DELAY * (1 + 0.5 * distance);  // scale delay for top moves
  int normalDelay = TIME_DELAY;                          // for up/down movements

  // go to top of src
  TOP[5] = pSrc[5];
  writeClawPWM(10);  // open claw early
  Braccio.moveTo(TOP[0], TOP[1], TOP[2], TOP[3], TOP[4], TOP[5]);
  delay(topMoveDelay);

  // descend to src
  Braccio.moveTo(pSrc[0], pSrc[1], pSrc[2], pSrc[3], pSrc[4], pSrc[5]);
  delay(normalDelay);
  writeClawPWM(40);  // close claw
  delay(800);

  // lift back to top
  Braccio.moveTo(TOP[0], TOP[1], TOP[2], TOP[3], TOP[4], TOP[5]);
  delay(normalDelay);

  // move to top of dst
  TOP[5] = pDst[5];
  writeClawPWM(40);  // keep claw closed
  Braccio.moveTo(TOP[0], TOP[1], TOP[2], TOP[3], TOP[4], TOP[5]);
  delay(topMoveDelay);

  // descend to dst
  Braccio.moveTo(pDst[0], pDst[1], pDst[2], pDst[3], pDst[4], pDst[5]);
  delay(normalDelay);
  writeClawPWM(10);  // open claw
  delay(800);

  // lift again
  Braccio.moveTo(TOP[0], TOP[1], TOP[2], TOP[3], TOP[4], TOP[5]);
  delay(normalDelay);

  Serial.println("Done");
}

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  pinMode(SERVO_PIN, OUTPUT);

  if (Braccio.begin()) {
    Serial.println("Braccio ready.");
    Serial.println("Input formats:");
    Serial.println(" - 80,120,125,50,10 : Direct pose override");
    Serial.println(" - A : Go to position A (A-F)");
    Serial.println(" - A,C : Move from A → C safely");
    Serial.println(" - t : move to top, c : close claw, o : open claw");
    moveArm(TOP, clawAngle);
  }
}

// ------------------ Loop ------------------
void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  // Direct numbers (5-number override)
  if (input.indexOf(',') > 0 && isDigit(input[0])) {
    int values[5], idx = 0, last = 0;
    for (int i = 0; i < input.length() && idx < 5; i++) {
      if (input[i] == ',' || i == input.length() - 1) {
        int end = (i == input.length() - 1) ? i + 1 : i;
        values[idx++] = input.substring(last, end).toInt();
        last = i + 1;
      }
    }
    if (idx == 5) {
      currentPose[2] = values[0];
      currentPose[3] = values[1];
      currentPose[4] = values[2];
      currentPose[5] = values[3];
      moveArm(currentPose, values[4]);
    }
  }
  // Move from one letter to another safely (A,C)
  else if (input.length() == 3 && input[1] == ',') {
    char src = toupper(input[0]);
    char dst = toupper(input[2]);
    moveObject(src, dst);
  }
  // Single letters (A-F) or special commands
  else if (input.length() == 1) {
    char c = toupper(input[0]);
    switch (c) {
      case 'A': case 'B': case 'C': case 'D': case 'E': case 'F': {
        float* pose = getPose(c);
        if (pose) moveArm(pose);
        break;
      }
      case 'X': {  // top
        moveArm((float*)FIXED_TOP);
        break;
      }
      case 'Y': {  // close claw
        writeClawPWM(40);
        Serial.println("Done");
        break;
      }
      case 'Z': {  // open claw
        writeClawPWM(10);
        Serial.println("Done");
        break;
      }
      case 'H': {  // wave claw
        for (int i = 0; i < 3; i++) {
          writeClawPWM(40);  // close
          delay(600);
          writeClawPWM(10);  // open
          delay(600);
        }
        Serial.println("Done");
        break;
      }
      case 'L': { // top sequence loop
        Serial.println("Starting L-loop. Send any command to exit.");
        bool exitLoop = false;
        int sequence[6][5] = {
          {70, 95, 200, 0, 10},
          {200, 190, 180, 10, 40},
          {70, 95, 200, 40, 10},
          {110, 150, 180, 120, 40},
          {190, 130, 170, 100, 10},
          {130, 100, 150, 150, 20}
        };
        while (!exitLoop) {
          for (int i = 0; i < 6; i++) {
            Braccio.moveTo(0, sequence[i][0], sequence[i][1], sequence[i][2], sequence[i][3], 0);
            writeClawPWM(sequence[i][4]);
            delay(TIME_DELAY);
            if (Serial.available()) {
              Serial.readString(); // clear buffer
              exitLoop = true;
              Serial.println("Exiting L-loop.");
              break;
            }
          }
        }
        break;
      }
      default: {
        Serial.println("Invalid");
        break;
      }
    }
  } else {
    Serial.println("Invalid");
  }
}
