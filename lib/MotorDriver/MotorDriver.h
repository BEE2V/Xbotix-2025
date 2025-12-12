#ifndef ROBOFEST2023_MOTORDRIVER_H
#define ROBOFEST2023_MOTORDRIVER_H

class MotorDriver{
    private:
        int correctionMax = 15;

        int leftBase = 80;
        int rightBase = 80;

        int sonicLeftBase = 60;
        int sonicRightBase = 60;

        int leftMax = 200;
        int rightMax = 200;

        int leftPWM;
        int rightPWM;
        int leftDirection[2];
        int rightDirection[2];

    public:
        MotorDriver(int* leftPins,int *rightPins);

        void forward(int speed);
        
        void backward(int speed);

        void turnLeft(int leftSpeed, int rightSpeed);
        
        void turnRight(int leftSpeed, int rightSpeed);

        void reverseRight(int speed);

        void reverseLeft(int speed);

        void forward(int leftSpeed, int rightSpeed);

        void backward(int leftSpeed, int rightSpeed);

        void stop(int stopDelay = 1000);

        void brake();

        void applyEncoderPid(int correction, int base);

        void applyLinePid(int correction, int base = -1);

        void applyWallPid(int correction);

        void applySonicPid(int correction);
        
};

#endif //ROBOFEST2023_MOTORDRIVER_H