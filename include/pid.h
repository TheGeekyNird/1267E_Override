class PID{
    private:
        //Forward & Backward
        double kp_fb;
        double ki_fb;
        double kd_fb;
        //Turn
        double kp_tu;
        double ki_tu;
        double kd_tu;

        bool finish;
        int dt;
        double ratio;
        int minimum;
        int fb_max;
        int lw_max;
        int tu_max;
        int settle;
        double e_break;
        double a_break;


        void tare_prepare();
        void set_finished();
    public:
        PID(double kp_fb,double ki_fb,double kd_fb, double kp_tu,double ki_tu,double kd_tu, double dt=10);
        void go(double distangle, bool turn, int timeout=-1);

};