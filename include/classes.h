//PID Loops for Drivetrain
class DrivePID{
    private:
        //Forward & Backward
        double kp_fb;
        double ki_fb;
        double kd_fb;
        //Turn
        double kp_tu;
        double ki_tu;
        double kd_tu;

        //General
        bool finish;
        double breakpoint;
        double minimum;
        int dt;
        double target;
        bool turn;
        std::uint32_t time;

        //Loop
        double l_encode;
        double r_encode;
        double l_error;
        double r_error;
        double lp;
        double rp;
        double li;
        double ri;
        double loe;
        double roe;
        double ld;
        double rd;
        double l_add;
        double r_add;
        double l_motor;
        double r_motor;
        double kp;
        double ki;
        double kd;
        int t;
        //R
        int settle;
        int s;
        //R

        void hmove(double distance);
        void hturn(double angle);

    public:
        bool is_finished();
        DrivePID(double kp_fb,double ki_fb,double kd_fb, double kp_tu,double ki_tu,double kd_tu, double dt=20);
        void prepare(double distangle, bool turn=0, bool rev=0);
        void go();
        void timeout(int t);
        void q_go(double distangle, bool turn=0, int wait=0);
};

//On-Screen Buttons
class ScreenButton{
    private:
        int x;
        int y;
        int length;
        int height;
        int color_one;
        int color_two;
        bool toggle;
        bool enable;
        std::string str_one;
        std::string str_two;
        int border_one;
        int border_two;
        void draw_button();
    public:
        ScreenButton(int x, int y, int length, int width, int color_one, int color_two, std::string str_one, std::string str_two, int border_one=0x000000, int border_two=0xFFFFFF, bool toggled = 0);
        bool toggled();
        void change(bool value);
        void poll(int touch_x, int touch_y);
        void enabled(bool enable);
};