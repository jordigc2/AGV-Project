#include "ros.h"
#include "AGV5/target.h"

extern AGV5::target velocity_msg;

class MotorController{
	
	public:
		int mr_opins[3];
		int mr_ipins[2];

		int ml_opins[3];
		int ml_ipins[2];
		
		MotorController(float* k_p, float* k_d, float* k_i, ros::NodeHandle* nodehandle);
	
		void driveWheels(bool isRight, float vel);

		void isrR(int ct);
		void isrL(int ct);
		void resetState();
		int getR();
		int getL();

		void get_velocity(unsigned int* t);
		void getRpms(int count_r, int count_l, unsigned int prevTime);
		void pidWheels(float target, float rpm, bool mot, float& sig, float& last_e, float& sum_e);

		void reset(); 	
		void driver(float target_linvel, float target_angvel);

		ros::NodeHandle nh_;
	

	private:
	
		int stndby; 
		
		volatile int r_state = 0;
		volatile int l_state = 0;
		
		float vright = 0;
		float vleft = 0;
		

		int PULSE_PR_TURN;    

		unsigned int pTime = 0;

		float sig_r = 0;
		float sig_l = 0;

		float prevE_r = 0;
		float prevE_l = 0;

		float eSum_r = 0;
		float eSum_l = 0;

		float kp;
		float kd;
		float ki;
};
