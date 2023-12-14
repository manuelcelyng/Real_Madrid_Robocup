#ifndef DRIBBLEO_H
#define DRIBBLEO_H

#define MOTOR_CONTROL_1_PIN 16
#define MOTOR_CONTROL_2_PIN 17
#define MOTOR_ENABLE 28

class Dribbleo{
    public:
        Dribbleo();
        void activeDribbleo(bool value1, bool value2);
};

extern Dribbleo* dribbleo;

#endif