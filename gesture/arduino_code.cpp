const int LEFT_IN1 = 7;
const int LEFT_IN2 = 8;
const int LEFT_EN = 5; // PWM

const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 11;
const int RIGHT_EN = 6; // PWM

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 1400; // if no command for 1.4s -> stop

int curLeftSpeed = 0;
int curRightSpeed = 0;

// ramping parameters
const int RAMP_STEP = 8; // PWM step per loop
const int RAMP_DELAY_MS = 30;

void setup()
{
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(LEFT_EN, OUTPUT);

    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);

    Serial.begin(115200);
    stopMotorsImmediate();
    lastCmdTime = millis();
}

void stopMotorsImmediate()
{
    analogWrite(LEFT_EN, 0);
    analogWrite(RIGHT_EN, 0);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    curLeftSpeed = 0;
    curRightSpeed = 0;
}

void setLeft(int pwm)
{
    // pwm in -255..255
    pwm = constrain(pwm, -255, 255);
    if (pwm == 0)
    {
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, LOW);
        analogWrite(LEFT_EN, 0);
    }
    else if (pwm > 0)
    {
        digitalWrite(LEFT_IN1, HIGH);
        digitalWrite(LEFT_IN2, LOW);
        analogWrite(LEFT_EN, pwm);
    }
    else
    { // reverse
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, HIGH);
        analogWrite(LEFT_EN, -pwm);
    }
    curLeftSpeed = pwm;
}

void setRight(int pwm)
{
    pwm = constrain(pwm, -255, 255);
    if (pwm == 0)
    {
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, LOW);
        analogWrite(RIGHT_EN, 0);
    }
    else if (pwm > 0)
    {
        digitalWrite(RIGHT_IN1, HIGH);
        digitalWrite(RIGHT_IN2, LOW);
        analogWrite(RIGHT_EN, pwm);
    }
    else
    {
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, HIGH);
        analogWrite(RIGHT_EN, -pwm);
    }
    curRightSpeed = pwm;
}

String inputLine = "";

void applyRampedTargets(int targetL, int targetR)
{
    // ramp current curLeftSpeed -> target in steps
    while (curLeftSpeed != targetL || curRightSpeed != targetR)
    {
        if (curLeftSpeed < targetL)
            curLeftSpeed = min(curLeftSpeed + RAMP_STEP, targetL);
        else if (curLeftSpeed > targetL)
            curLeftSpeed = max(curLeftSpeed - RAMP_STEP, targetL);

        if (curRightSpeed < targetR)
            curRightSpeed = min(curRightSpeed + RAMP_STEP, targetR);
        else if (curRightSpeed > targetR)
            curRightSpeed = max(curRightSpeed - RAMP_STEP, targetR);

        setLeft(curLeftSpeed);
        setRight(curRightSpeed);
        delay(RAMP_DELAY_MS);
    }
}

void parseAndExecute(String cmd)
{
    // Expect: <CHAR><NUM>\n e.g. "F150\n" or "S0\n"
    if (cmd.length() < 1)
        return;
    char c = cmd.charAt(0);
    int val = 150; // default
    if (cmd.length() > 1)
    {
        String num = cmd.substring(1);
        val = num.toInt();
    }
    // map val 0..255 to target pwm (-255..255) depending on command
    int targetL = 0, targetR = 0;
    switch (c)
    {
    case 'F': // forward
        targetL = val;
        targetR = val;
        break;
    case 'B': // backward
        targetL = -val;
        targetR = -val;
        break;
    case 'L': // left in-place (left backward, right forward)
        targetL = -val;
        targetR = val;
        break;
    case 'R': // right in-place
        targetL = val;
        targetR = -val;
        break;
    case 'S': // stop
        targetL = 0;
        targetR = 0;
        break;
    default:
        // unknown, stop
        targetL = 0;
        targetR = 0;
    }
    applyRampedTargets(targetL, targetR);
}

void loop()
{
    // read serial until newline
    while (Serial.available() > 0)
    {
        char in = (char)Serial.read();
        if (in == '\n')
        {
            // process line
            parseAndExecute(inputLine);
            inputLine = "";
            lastCmdTime = millis();
        }
        else if (in >= 32)
        {
            inputLine += in;
            // protect line length
            if (inputLine.length() > 10)
                inputLine = "";
        }
    }

    // safety timeout: stop if no commands recently
    if (millis() - lastCmdTime > TIMEOUT_MS)
    {
        // ramp to zero
        applyRampedTargets(0, 0);
        lastCmdTime = millis(); // reset to avoid repeated ramps
    }
}
