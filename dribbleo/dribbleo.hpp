#ifndef DRIBBLEO_H
#define DRIBBLEO_H

#define MOTOR_CONTROL_1_PIN 1
#define MOTOR_CONTROL_2_PIN 2
#define MOTOR_ENABLE 3

class Dribbleo{
    public:
        Dribbleo();
        void activeDribbleo(bool value1, bool value2);
};

extern Dribbleo* dribbleo;


#endif