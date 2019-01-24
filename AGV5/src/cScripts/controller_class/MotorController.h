#include "ros.h"
#include "AGV5/target.h"

class MotorController{
	
	public:
	        int mr_opins[3];
		int mr_ipins[2];

		int ml_opins[3];
		int ml_ipins[2];
		
		MotorController(float* k_p, float* k_d, float* k_i, ros::NodeHandle* nodehandle);
	
		void driveRight(int vel, bool dir);
		void driveLeft(int vel, bool dir);     
		
		void isrR();
		void isrL();
		
		void getRpms(int count_r, int count_l, unsigned long prevTime);
		void pidWheels(float target, float rpm, bool mot, int& sig, float& last_e, float& sum_e);


                void reset(); 	
		void driver(float target_linvel, float target_angvel, unsigned int& t);


                ros::NodeHandle nh_;
	

	private:
	
		int stndby; 
		
		unsigned int r_state = 0;
		unsigned int l_state = 0;
		
		float vright = 0;
		float vleft = 0;
		

		int PULSE_PR_TURN;    

		unsigned int pTime = 0;

		int sig_r = 0;
		int sig_l = 0;

		float prevE_r = 0;
		float prevE_l = 0;

		float eSum_r = 0;
		float eSum_l = 0;

		float kp;
		float kd;
		float ki;
};
