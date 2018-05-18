class PID
{
public:
    struct sData
    {
        sData()
            : m_reference(0.f), m_lastError(0.f), m_sumError(0.f), m_diffError(0.f)
        {
        }

        float m_reference;

        float m_lastError;
        float m_sumError;
        float m_diffError;
    };

private:
    float m_Kp;
    float m_Ki;
    float m_Kd;

    sData m_data;

public:
    PID()
        : m_Kp(0.f), m_Ki(0.f), m_Kd(0.f), m_data()
    {
    }

    void setParameters(float Kp, float Ki, float Kd)
    {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
    }

    void setReference(float r)
    {
        m_data.m_reference = r;
    }

    const char* data() const
    {
        return reinterpret_cast<const char *>(&m_data);
    }

    float update(float measurement)
    {
        const float err = m_data.m_reference - measurement;

        m_data.m_diffError = m_data.m_lastError - err;
        m_data.m_sumError += err;
        m_data.m_lastError = err;

        return (m_Kp * err) + (m_Ki * m_data.m_sumError) + (m_Kd * m_data.m_diffError);
    }
};

struct sMotorPin
{
    byte enc;
    byte fwd;
    byte bwd;
    byte pwm;
};

typedef void(*func_type)(void);

enum eMotorSide
{
    RIGHT,
    LEFT,
    MOTOR_NUM
};

#pragma pack(push, 1)
struct sCommand
{
    uint8_t fwd[MOTOR_NUM];
    float distance[MOTOR_NUM];
};
#pragma pack(pop)

const sMotorPin pins[MOTOR_NUM] = {
  { 18, 7, 8, 10 }, // RIGHT
  { 19, 2, 4, 9 }   // LEFT
};

const int32_t timerStart = 59286; // 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
const float encoder2Speed = 0.31416f;
const int32_t ticksPerRevolution = 2240;
const float distanceToRev = 1.f / (100.f * PI);
const float Te = 0.1f;
const float invTe = 10.f; // 1/Te
const float speedLimit = 0.3f; // [m/s]

const byte feedbackPin = 0;

int32_t encoder[MOTOR_NUM] = { 0, 0 };
float distance[MOTOR_NUM] = { 0.f, 0.f };
float speed[MOTOR_NUM] = { 0.f, 0.f };
PID pidMotor[MOTOR_NUM];
PID pidDistance[MOTOR_NUM];

void encoderInterruptRight()
{
    ++encoder[RIGHT];
}

void encoderInterruptLeft()
{
    ++encoder[LEFT];
}

const func_type encoderInterrupts[MOTOR_NUM] = {
    encoderInterruptRight,
    encoderInterruptLeft
};

float minmax(float min, float value, float max)
{
    return (value < min) ? min : ((value > max) ? max : value);
}

ISR(TIMER1_OVF_vect)
{
    // reset timer
    TCNT1 = timerStart;

    for (byte m = 0; m < MOTOR_NUM; ++m)
    {
        const float dist = encoder[m] * encoder2Speed / ticksPerRevolution;
        speed[m] = dist * invTe; // V = d / T
        distance[m] += dist;
    }

    for (byte m = 0; m < MOTOR_NUM; ++m)
    {
        float refSpeed = pidDistance[m].update(distance[m]);
        refSpeed = minmax(0.f, refSpeed, speedLimit);

        pidMotor[m].setReference(refSpeed);

        const float pwm = pidMotor[m].update(speed[m]);
        analogWrite(pins[m].pwm, minmax(0.f, pwm, 80.f));
    }

    memset(&encoder, 0, sizeof(encoder));
}

void setDirection(byte side, bool fwd)
{
    digitalWrite(pins[side].fwd, fwd);
    digitalWrite(pins[side].bwd, !fwd);
}

void stopMotor(byte side)
{
    digitalWrite(pins[side].fwd, LOW);
    digitalWrite(pins[side].bwd, LOW);
    analogWrite(pins[side].pwm, 0);
}

void setup()
{
    Serial.begin(115200);
    pidMotor[RIGHT].setParameters(80.f, 30.f, 0.f);
    pidMotor[LEFT].setParameters(80.f, 30.f, 0.f);
    pidDistance[RIGHT].setParameters(1.f, 0.f, 0.f);
    pidDistance[LEFT].setParameters(1.f, 0.f, 0.f);

    for (byte m = 0; m < MOTOR_NUM; ++m)
    {
        pinMode(pins[m].enc, INPUT_PULLUP);
        pinMode(pins[m].fwd, OUTPUT);
        pinMode(pins[m].bwd, OUTPUT);
        pinMode(pins[m].pwm, OUTPUT);
        attachInterrupt(digitalPinToInterrupt(pins[m].enc), encoderInterrupts[m], CHANGE);
    }

    pinMode(feedbackPin, OUTPUT);
    digitalWrite(feedbackPin, LOW);

    noInterrupts();

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = timerStart;       // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

    pidDistance[RIGHT].setReference(0.f);
    pidDistance[LEFT].setReference(0.f);

    setDirection(RIGHT, true);
    setDirection(LEFT, true);

    interrupts();
}

const float targetDistance = 1.f; // [m]
const float halfRotation = 11.8f * PI * 0.01f; // [m]
float crtTarget = 0.f;
void loop()
{
    bool bDistanceReached = false;
    for (byte m = 0; m < MOTOR_NUM; ++m)
    {
        bDistanceReached |= (distance[m] >= crtTarget);
    }

    if (bDistanceReached)
    {
        memset(&distance, 0, sizeof(distance));
        crtTarget = 0.f;
        for (byte m = 0; m < MOTOR_NUM; ++m)
        {
            pidDistance[m].setReference(crtTarget);
        }

        digitalWrite(feedbackPin, LOW);
    }

    if ((Serial.available() >= sizeof(sCommand)) && bDistanceReached)
    {
        digitalWrite(feedbackPin, HIGH);
        
        sCommand command;
        Serial.readBytes(reinterpret_cast<char *>(&command), sizeof(command));

        for (byte m = 0; m < MOTOR_NUM; ++m)
        {
            setDirection(m, command.fwd[m] == 1);
            pidDistance[m].setReference(command.distance[m]);
            crtTarget = command.distance[m];
        }
    }
}
