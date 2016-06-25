 #include <ros/ros.h>
 #include <hexapode_v2/translation.h>
 #include <signal.h>
 #include <termios.h>
 #include <stdio.h>

 #define KEYCODE_UP 0x41
 #define KEYCODE_LEFT 0x44
 #define KEYCODE_DOWN 0x42
 #define KEYCODE_RIGHT 0x43


 class keyListener
 {
 public:
   keyListener();
   void keyLoop();

 private:
   ros::NodeHandle nh_;
   float dx, dy, dh, da;
   ros::Publisher trans_pub_;

 };

 keyListener::keyListener():
   dx(0),
   dy(0),
   dh(0),
   da(0)
 {

   trans_pub_ = nh_.advertise<hexapode_v2::translation>("translation", 100);
 }

 int kfd = 0;
 struct termios cooked, raw;

 void quit(int sig)
 {
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
 }


 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "keyListener");

   keyListener listener;
   signal(SIGINT,quit);
   listener.keyLoop();

   return(0);
 }


 void keyListener::keyLoop()
 {
   char c;
   bool dirty=false;


   // get the console in raw mode
   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   // Setting a new line, then end of file stages
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);

   puts("Reading from keyboard");
   puts("---------------------------");
   puts("Use arrow keys to move the hexapode.");


   for(;;)
   {
     // get the next event from the keyboard
     if(read(kfd, &c, 1) < 0)
     {
       perror("read():");
       exit(-1);
     }

     dx = 0;
     dy = 0;
     dh = 0;
     da = 0;

     ROS_DEBUG("value: 0x%02X\n", c);

     switch(c)
     {
       case KEYCODE_UP:
         ROS_DEBUG("UP");
         dx += 0;
         dy += 1;
         dh += 0;
         da += 0;
         dirty = true;
         break;
       case KEYCODE_DOWN:
         ROS_DEBUG("DOWN");
         dx += 0;
         dy += -1;
         dh += 0;
         da += 0;
         dirty = true;
         break;
       case KEYCODE_LEFT:
         ROS_DEBUG("LEFT");
         dx += -1;
         dy += 0;
         dh += 0;
         da += 0;
         dirty = true;
         break;
       case KEYCODE_RIGHT:
         ROS_DEBUG("RIGHT");
         dx += 1;
         dy += 0;
         dh += 0;
         da += 0;
         dirty = true;
         break;
	  case 'u':
         ROS_DEBUG("UP");
         dx += 0;
         dy += 0;
         dh += 1;
         da += 0;
         dirty = true;
         break;
	  case 'd':
         ROS_DEBUG("DOWN");
         dx += 0;
         dy += 0;
         dh += -1;
         da += 0;
         dirty = true;
         break;
	  case 'f':
         ROS_DEBUG("Aiguille");
         dx += 0;
         dy += 0;
         dh += 0;
         da += 1;
         dirty = true;
         break;
	  case 'j':
         ROS_DEBUG("Trigo");
         dx += 0;
         dy += 0;
         dh += 0;
         da += -1;
         dirty = true;
         break;

     }


     hexapode_v2::translation trans;

     trans.dx = dx;
     trans.dy = dy;
     trans.da = da;
     trans.dh = dh;

        ROS_DEBUG("dx: [%f]", trans.dx);
        ROS_DEBUG("dy: [%f]", trans.dy);
        ROS_DEBUG("da: [%f]", trans.da);
        ROS_DEBUG("dh: [%f]", trans.dh);

     if(dirty ==true)
     {
       trans_pub_.publish(trans);
       dirty=false;
     }
   }


   return;
 }
