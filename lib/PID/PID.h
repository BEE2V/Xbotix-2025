#ifndef ROBOFEST2023_PID_H
#define ROBOFEST2023_PID_H

class PID{
    private:
        //line follow with velocity control
        const double vP = 0.03;  //0.025
        const double vI = 0.0;
        const double vD = 0.05;  //0.05

        int prevLinevError = 0;
        int totalLinevError = 0;

        const double veP = 10.0;
        const double veI = 0.0;
        const double veD = 0.0;

        int prevEncvError = 0;
        int totalEncvError = 0;

        //line follow with normal control
        const double P = 0.03;  //0.016
        const double I = 0.0;
        const double D = 0.015;  //0.062

        int prevLineError = 0;
        int totalLineError = 0;

        const double eP = 10;
        const double eI = 0.0;
        const double eD = 0.0;

        int prevEncError = 0;
        int totalEncError = 0;

        //wall follow with ultrasonic
        const double wP = 0.8;
        const double wI = 0;    
        const double wD = 1;

        const double sP = 0.8;   
        const double sI = 0;    
        const double sD = 1;


    public: 

        int prevWallError = 0;
        
        int prevSonicError = 0;


        float getLineCorrection(int err);

        float getEncoderCorrection(int err);
        

        float getLineCorrectionVC(int err);

        float getEncCorrectionVC(int err);


        int getWallCorrection(int err);

        int getSonicCorrection(int err);

    
};

#endif //ROBOFEST2023_PID_H