#include<Arduino.h>
// #include<TimerOne.h>

class motorSys {
    public:
        motorSys(int a);
        int getdata();
        void SerialIni(int baudrate);
        void initializeport(int m1, int m2, int ena, int enb);
        void publish_data();
        void control_loop();
        void setup_timer_interrupt(unsigned long dt_us, unsigned int prescaler);
        double get_rpm();
        static motorSys* _instance;

        static void enc_ISR_wrapper();

    private:
        int x;
        int _encoder_count;
        double _rpm;        
        void enc_ISR();
};
