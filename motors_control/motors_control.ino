class PID
{
    private:
        float m_Kp;
        float m_Ki;
        float m_Kd;

        float m_reference;

        float m_lastError;
        float m_sumError;

    public:
        PID()
            : m_Kp(0.f), m_Ki(0.f), m_Kd(0.f), m_reference(0.f), m_lastError(0.f), m_sumError(0.f)
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
            m_reference = r;
        }

        float update(float measurement)
        {
            const float err = m_reference - measurement;

            const float diffError = m_lastError - err;
            m_sumError += err;
            m_lastError = err;

            return (m_Kp * err) + (m_Ki * m_sumError) + (m_Kd * diffError);
        }
};

struct sMotorPin
{
    byte enc;
    byte fwd;
    byte bwd;
    byte pwm;
};

typedef void (*func_type)(void);

const byte MOTORS = 2;
const sMotorPin pins[MOTORS] = {
    { 18, 7, 8, 10 },
    { 19, 2, 4, 9 }
};

const int32_t timerStart = 59286; // 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
const float encoder2Speed = 0.31416f;
const int32_t ticksPerRevolution = 2240;
const float distanceToRev = 1.f / (100.f * PI);
const float Te = 0.1f;
const float invTe = 10.f; // 1/Te

int32_t encoder[MOTORS] = { 0, 0 };
float distance[MOTORS] = { 0.f, 0.f };
float speed[MOTORS] = { 0.f, 0.f };
PID pidMotor[MOTORS];

void encoderInterrupt0()
{
    ++encoder[0];
}

void encoderInterrupt1()
{
    ++encoder[1];
}

const func_type encoderInterrupts[MOTORS] = {
    encoderInterrupt0,
    encoderInterrupt1
};

float minmax(float min, float value, float max)
{
    return (value < min) ? min : ((value > max) ? max : value);
}

ISR(TIMER1_OVF_vect)
{
    for (byte m = 0; m < MOTORS; ++m)
    {
        speed[m] = encoder[m] * encoder2Speed / ticksPerRevolution * invTe;
    }

    memset(&encoder, 0, sizeof(encoder));

    const float pwm[MOTORS] = {
        pidMotor[0].update(speed[0]),
        pidMotor[1].update(speed[1])
    };

    for (byte m = 0; m < MOTORS; ++m)
    {
        analogWrite(pins[m].pwm, minmax(0.f, pwm[m], 255.f));
    }

    for (byte m = 0; m < MOTORS; ++m)
    {
        distance[m] += speed[m] * Te;
    }

    // reset timer
    TCNT1 = timerStart;
}

void startMotor(int id, bool fwd, bool bwd)
{
    digitalWrite(pins[id].fwd, fwd);
    digitalWrite(pins[id].bwd, bwd);
}

void stopMotor(int id)
{
    digitalWrite(pins[id].fwd, LOW);
    digitalWrite(pins[id].bwd, LOW);
    analogWrite(pins[id].pwm, 0);
}

byte recData[(3 * sizeof(byte)) + sizeof(float)];
void onReceive()
{
    if (Serial.available() >= sizeof(recData))
    {
        byte id = 0;
        byte fwd = 0;
        byte bwd = 0;
        float speed = 0;
        Serial.readBytes(&recData[0], sizeof(recData));
        memcpy(&id, &recData[0], sizeof(id));
        memcpy(&fwd, &recData[sizeof(id)], sizeof(fwd));
        memcpy(&bwd, &recData[sizeof(id)+sizeof(fwd)], sizeof(bwd));
        memcpy(&speed, &recData[sizeof(id)+sizeof(fwd)+sizeof(bwd)], sizeof(speed));
        
        // set direction
        startMotor(id, fwd, bwd);
        pidMotor[id].setReference(speed);
    }
}

byte data[sizeof(float) * 2];
void onRequest() // I2C
{
    memcpy(&data[0], &speed[0], sizeof(float));
    memcpy(&data[sizeof(float)], &speed[1], sizeof(float));
    Serial.write(data, sizeof(data));
}

void setup()
{
    Serial.begin(115200);
    pidMotor[0].setParameters(50.f, 30.f, 0.f);
    pidMotor[1].setParameters(50.f, 30.f, 0.f);

    for (byte m = 0; m < MOTORS; ++m)
    {
        pinMode(pins[m].enc, INPUT_PULLUP);
        pinMode(pins[m].fwd, OUTPUT);
        pinMode(pins[m].bwd, OUTPUT);
        pinMode(pins[m].pwm, OUTPUT);
        attachInterrupt(digitalPinToInterrupt(pins[m].enc), encoderInterrupts[m], CHANGE);
    }

    noInterrupts();

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = timerStart;       // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

    pinMode(21, INPUT);

    interrupts();
}

const float targetDistance = 0.5f; // [m]
const float halfRotation = 12.5f * PI * 0.01f / 2.f; //[m]
float crtTarget = 0.4f;
byte state = 0;
void loop()
{
    if (digitalRead(21) == 0)                                                                                                                                                                                                                                                                                                                                                                      
    {
        if (state == 0)
        {
            startMotor(0, true, false);
            startMotor(1, true, false);

            pidMotor[0].setReference(0.2f);
            pidMotor[1].setReference(0.2f);
            state = 1;
        }
        
        bool bDistanceReached = false;
        for (byte m = 0; m < MOTORS; ++m)
        {
            bDistanceReached |= distance[m] > crtTarget;
        }
    
        if (bDistanceReached)
        {
            memset(&distance, 0, sizeof(distance));
            startMotor(0, false, true);
            startMotor(1, false, true);
            delayMicroseconds(500);
            stopMotor(0);
            stopMotor(1);

            ++state;

            switch(state)
            {
                case 2:
                {
                    crtTarget = halfRotation;
                    startMotor(0, false, true);
                    startMotor(1, true, false);
                }
                break;
                case 3:
                {
                    crtTarget = 0.55f;
                    startMotor(0, true, false);
                    startMotor(1, true, false);
                }
                break;
                case 4:
                {
                    crtTarget = halfRotation;
                    startMotor(0, false, true);
                    startMotor(1, true, false);
                }
                break;
                case 5:
                {
                    crtTarget = 0.5f;
                    startMotor(0, true, false);
                    startMotor(1, true, false);
                }
                break;
            }
        }
    }
}
