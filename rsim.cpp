//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdlib.h"
#include "string.h"
#include <mutex>
#include <time.h>
#include <stdio.h>
#include <tchar.h>
#include <string>
//
#include "SerialClass.h"
#include "Serial.h"
#include "compress12bit.h"

//for file input
#include <array>
#include<fstream>
#include <string>
#include <sstream>

//for keyboard input
#include <iostream>
#include <conio.h>
#include <ctype.h>

//-------------------------------- SERIAL PORT COMMUNICATION -------------------------------------
char incomingData[5] = "0000";			// don't forget to pre-allocate memory
int readResult = 0;
int timeout_counter = 0;
float forceIn;
float forcei;
float posOuti = 2;// make consistent with Arduino code
float posOut_prev = 2;
bool run_mode = true;
int run_mode_counter = 0;
bool SRlatch_Q = false;
bool SRlatch_nQ = true;
bool SRlatch_Q_prev = false;
bool SRlatch_nQ_prev = true;
bool is_unstable = false;
bool is_overextended = false;
float liveforce = 0;
float forceAdj = 0;
float lrest = 0; // length of muscle from actual prep, not to be confused with Li which is initial length of model's mtu path
float forceScale = 1; // scales Aurora force if too high then compensates in MJ
float forcePrev = 0;
float posOut;
float lf = 0;
char sendbuf[3]; // for position data out (rs232)
Serial* SP = new Serial("\\\\.\\COM1");    // initialize serial port

//----------------------------------------------------------------------------------------


//-------------------------------- global variables -------------------------------------

// synchronization
std::mutex gui_mutex;
char lastfile[1000] = "";

char error[1000];
// model
//mjModel* m = 0;
mjModel* m = mj_loadXML("5SegRM.xml", NULL, error, 1000);
mjData* d = 0;

//mjModel* m;
//mjData* d;
//save data
char const *outfile = "savedData.txt";
// char const *outfile_kinematics = "savedData_kinematics.txt";
//allocate array for output data
long double arrayout[10000] = {0};
float dataout[10000][20];
float pos_dataout[10000][27];

int number_of_samples = 300;
int counter = 0;
clock_t tStart = clock();
float realTime = 0;
float prevTime = 0;

float li = 0; //initial tendon length

std::array<std::array<float, 11>, 1> inputdata_qpos;//i rows, j cols
std::array<std::array<float, 4>, 1000> inputdata_inforces;//3 input forces, 1000 rows
std::ifstream datfile_qpos("qposInit.dat");
std::ifstream datfile_forceScale("forceScale.dat");
std::ifstream datfile_restLength("lrest_m.dat");
std::ifstream datfile_inforces("inforces.dat");

char opt_title[1000] = "";
char opt_content[1000];


void SRlatch(bool latchSET, bool latchRESET)
{
	bool SRenable = true;
	bool SRlatch_Q_prev = SRlatch_Q;
	bool SRlatch_nQ_prev = SRlatch_nQ;
	SRlatch_Q = !(latchRESET||SRlatch_nQ_prev);
	SRlatch_nQ = !(latchSET||SRlatch_Q_prev);
	SRlatch_Q_prev =SRlatch_Q;
	SRlatch_nQ_prev =SRlatch_nQ;
}


// generate sinusoidal control waveform
void setcontrol(mjtNum time, mjtNum* ctrl, int nu)
{
    int switches[2] = {1};
	float force_offset = forcei;
	forceAdj = -liveforce * (1/ forceScale) * (forceIn - force_offset);
	if (forceAdj > 0) {
		forceAdj = 0;
	}

	
	//ctrl[0] = forceAdj;//ankle extensor
/* 	if (d->qpos[8] + m->qpos0[8] < -3) {// check if ankle angle is greater than 180 degrees, negative angle
		//ctrl[0] = inputdata_inforces[counter][0];//ankle extensor force
		ctrl[0] = forceAdj;//ankle extensor
		//printf("%f\n", forceAdj);
	}
	else {
		ctrl[0] = 0;
	} */
	ctrl[0] = forceAdj;//ankle extensor
	ctrl[1] = inputdata_inforces[counter][1];//knee extensor servo pos
	ctrl[2] = inputdata_inforces[counter][2];//hip extensor servo pos
	
	 // --- apply force to COM from forelimbs ---
    d->xfrc_applied[6*6 + 2] = inputdata_inforces[counter][3]; // body 6 (from 0) is com
	
	forcePrev = forceIn;
	//printf("%f\n", ctrl[0]);
    // for( int i=0; i<nu; i++ )
        //qfrc_applied[i] = mju_sin(time);
        //qfrc_applied[i] = -1* switches[i];
		
        // ctrl[i] = -(forceIn - 0.1);
		//printf("%f\n", forceAdj);
}

// simulation
void advance(void)
{
	forceIn = (decompress12bit(incomingData));
	
	
	
	//set initial pose
/* 	if (counter == 0) {
		for (int i = 0; i<=11; i++) {
			printf("qpos %f\n", d->qpos[i]);
		}
	} */
	if (counter > 0) {// allow simulation to free fall for first step to make sure model is in correct posture
		setcontrol(d->time, d->ctrl, m->nu);
	}
    
	// note mysterioous 4 ms delay before communication works properly
	//- you can see this if you make initial arduino write to be something other than posOuti, line 136
	//if (counter > 4) {
	//	mj_step(m, d);
	//} 
    mj_step(m, d);

	if (counter == number_of_samples - 1) {
		lf = d->sensordata[0];
		
	}
	if (counter == 0) {
		forcei = forceIn;
		mj_tendon(m, d);
		li = d->ten_length[0];
		SRlatch(false, true);
		//compress12bit(sendbuf, posOuti); 
	}


    counter ++;
	
	
	incomingData[readResult] = 0;
	mj_tendon(m, d);
	//posOut_prev = posOut;
	
	is_unstable = abs(posOut - posOut_prev) > 0.2;
	is_overextended = d->qpos[8] + m->qpos0[8] < -3;
	run_mode = !((is_unstable || is_overextended) && counter > 10);
	if (counter < 300) {
		//printf("unstable %d overextend %d posOut %f counter %i\n", is_unstable, is_overextended, posOut, counter);
		//printf("latch test %d  counter %i\n", SRlatch_Q, counter);
	}
	
	
	// NOTE: initial tend length defined as ML
	// subtract some initial value near top of the range, e.g. 2, so muscle has some room to lengthen first
	
	if (run_mode) { // only update posOut if not in safe mode
		if (run_mode_counter == 0) {
			SRlatch(true, false);
		}
		if (!SRlatch_Q) {
			posOut = posOuti - (- (d->ten_length[0] - li) / lrest) * (2.2 / 0.32); //tendon length multiplied and offset to be near max of DUE dac output limit 2.2 V range for 0.32 strain
			
			//Clamp posOut
/* 			if (posOut > 2.2) {
				posOut = 2.2;
			}
			else if (posOut < 0) {
				posOut = 0;
			} */
			if (posOut > 2.2 || posOut < 0) {
				posOut = posOut_prev;
			}
			
			posOut_prev = posOut;
		}
		else {
			//posOut = posOut_prev;
		}
		run_mode_counter ++;
/* 		//Clamp posOut
		if (posOut > 2.2) {
			posOut = 2.2;
		}
		else if (posOut < 0) {
			posOut = 0;
		} */
	}
	else {
		run_mode_counter = 0;
		
	}
	

	
	
	
	//posOut = forceIn; // for loopback testing
	//printf("%i   %f\n", counter, posOut);

	if (counter == 1) {
		printf("Li is the following value = %f\n", li);
		printf("Lrest = %f\n", lrest);
		printf("posOut is the following value = %f\n", posOut);
		printf("force scale %f\n", forceScale);
	}	
	

	compress12bit(sendbuf, posOut); //second joint angle
/* 	if (counter < 20) {
		compress12bit(sendbuf, posOuti);
	}
	else {
		compress12bit(sendbuf, posOut);
	} */
	
	sendbuf[2] = 65;
	SP->WriteData(sendbuf, sizeof(sendbuf));

	prevTime = realTime;
    realTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;//calculate real time sim takes
	// timeout_counter = 0;
	// while ((realTime - prevTime) == 0 && timeout_counter < 3000) {
		// timeout_counter ++;//wait
	// }


    arrayout[counter] = d->time;
    dataout[counter][0] = d->time;
    dataout[counter][1] = realTime;
	dataout[counter][2] = posOut;
	dataout[counter][3] = forceAdj;
	//dataout[counter][]
    for (int j = 0; j < (m->nbody) * 3; j++) {
		pos_dataout[counter][j] = d->xpos[j];//skip world body
	}
	
	for (int j =0; j < 3; j++) {
		pos_dataout[counter][j + (m->nbody) * 3] = d->geom_xpos[(m->ngeom - 1) * 3 + j];
	}
        


}



//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
    int maxsteps = 1000;
	// char input;
	// using namespace std;
	// cout << "hellofdgdf\n";
	// cin >> input;
	int ch = 89;
	_cputs("Press 'n' to ignore live force data\notherwise press any other key to start the simulation");

	while (ch == 89) {
		ch = _getch();
		if (ch == 110) {
			//n key
			liveforce = 0;
		}
		else {
			liveforce = 1;
		}
	}

    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    mj_activate("mjkey.txt");
    
    // install control callback
    //mjcb_control = mycontroller;

    
    //glfwSetDropCallback(window, drop);
   

    //loadmodel(window, "nothing.nothing", 0);


    mjModel* mnew = 0;
    
    mnew = mj_loadXML("5SegRM.xml", 0, error, 1000);

    
    {
        printf("%s\n", error);

    }
/* 	//load initial pose data
    for (int i = 0; i <= 1; i++) {
        for (int j = 0; j < 11; j++) {
            datfile_qpos >> inputdata_qpos[i][j];
        }
    }

    // inforces is ankle actuator force, knee servo position, hip servo position, forelimb force
    for (int i = 0; i < number_of_samples; i++) {
        for (int j = 0; j < 4; j++) {
            datfile_inforces >> inputdata_inforces[i][j];
        }
    }	 */
	
    // Get number of time samples
    number_of_samples = 0;
    std::string line;
    while (std::getline(datfile_inforces, line)) {
        ++number_of_samples;
    }
    datfile_inforces.close ();
    std::ifstream datfile_inforces("inforces.dat");

    for (int i = 0; i <= 1; i++) {
        for (int j = 0; j < 11; j++) {
            datfile_qpos >> inputdata_qpos[i][j];
        }
    }

    // inforces is ankle actuator force, knee servo position, hip servo position, forelimb force
    for (int i = 0; i < number_of_samples; i++) {
        for (int j = 0; j < 4; j++) {
            datfile_inforces >> inputdata_inforces[i][j];
        }
    }

    datfile_inforces.close ();
	datfile_forceScale >> forceScale;
	datfile_restLength >> lrest;
	
    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    //m = mj_loadXML("hello.xml", NULL, error, 1000);
    d = mj_makeData(m);
    mj_step(m, d);



    printf("fool...\n");
	printf("nq %i\n", m->nq);
	printf("nv %i\n", m->nv);
	printf("nbody %i\n", m->nbody);
	printf("nv %i\n", m->nv);
	printf("nsamples %i\n", number_of_samples);
	//mju_copy(d->qpos, m->key_qpos, m->nq*1);// tells MJ to load keyframe angles!!
	//---MUST set qpos0 as reference position.  MJ ignores the free joint, so skip that
    for (int i = 0; i < m->nq; i++) {
        m->qpos0[i] = inputdata_qpos[0][i];
    }   
    //now account for free joint
    for (int i = 0; i < 7; i++) {
        d->qpos[i] = inputdata_qpos[0][i];
    }
    printf("pos ref\n");
    for (int i = 0; i < m->nq; i++) {
        printf("%f\n", m->qpos0[i]);
    }

    for (int i = 0; i < m->nq; i++) {
        m->qpos_spring[i] = m->qpos0[i];
    }

    printf("spring ref\n");
    for (int i = 0; i < m->nq; i++) {
        printf("%f\n", m->qpos_spring[i]);
    }
	//advance();
	//mj_tendon(m, d);
	//li = d->ten_length[0];
	//printf("Li = %f\n", li);
    // main loop
    
    tStart = clock();
	
	
    // while( counter <= maxsteps )
    // {
        // advance();  

    // }
	if (SP->IsConnected()) {
		printf("We're connected\n");
		// printf("Five\n");
		// Sleep(1000);
		// printf("Four\n");
		// Sleep(1000);
		// printf("Three\n");
		// Sleep(1000);
		// printf("Two\n");
		// Sleep(1000);
		// printf("One\n");
		// Sleep(1000);
	  }
	 int reset_steps = 190; // number of steps over which to reset to initial posOut
	 float posReset = posOut;
	 while (SP->IsConnected() && counter < number_of_samples + reset_steps) 
	 {
		while (SP->ReadData(incomingData, 3) == 3 && incomingData[2] == 65)
		{
			timeout_counter = 0;
			//run mode
			if (counter < number_of_samples) {
				advance();
			}
			//reset mode
			//RESET to initial offset value so AURURA doesn't jump, but do it slowly!!
			else if (counter >= number_of_samples && counter < number_of_samples + reset_steps) {
				for (int i = 0; i < reset_steps; i ++) {
					float resetFraction = ((float)i / ((float)reset_steps - 1));
					posReset = posOuti * resetFraction + posOut * (1 - resetFraction);
					compress12bit(sendbuf, posReset); //second joint angle
					sendbuf[2] = 65;
					SP->WriteData(sendbuf, sizeof(sendbuf));
					Sleep(1);
					counter ++;
					//printf("posOut reset %f \n", posReset);
				}	
			}
		}
	 }

	
    //mj_printData(m, d, outfile);
    printf("quat1 %f %f %f %f\n", d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3]);
    printf("%d %f\n", counter, d->time);
    printf("%d\n", m->nu);
    printf("size %d \n", m->nq);

    FILE *f = fopen("savedData.txt", "w");//a for append, t for text mode
    //fwrite(arrayout, sizeof(char), sizeof(arrayout), f);
    
    for(int i = 1; i < number_of_samples; i++) {
        for (int j = 0; j < m->nq + 4; j++)
           fprintf(f, "%f\t", dataout[i][j]); 
        fprintf(f, "\n");

        //fprintf(f, "%Lf\n", arrayout[i]);
    }
	fclose(f);
	
	FILE *f1 = fopen("savedData_kinematics.txt", "w");//a for append, t for text mode
    //fwrite(arrayout, sizeof(char), sizeof(arrayout), f);
    
    for(int i = 1; i < number_of_samples; i++) {
        for (int j = 0; j < (m->nbody) * 3 + 3; j++)
           fprintf(f1, "%f\t", pos_dataout[i][j]); 
        fprintf(f1, "\n");

    }
	fclose(f1);
	
	// char incomingData[256] = "";	
	// int counter = 0;
	// Serial* SP = new Serial("\\\\.\\COM1");    // initialize serial port
	 // if (SP->IsConnected()) {
		// printf("We're connected\n"); 
	 // }
	// else {
		// printf(".....!NOT CONNECTED!....");
	// }	
	// while (SP->ReadData(incomingData, 1) > 0 && counter < 1000) {
		// counter ++;
	// }
	

	


	// if (SP->IsConnected()) {
		// printf("We're connected\n"); 
	 // }
	 // while (SP->IsConnected() && scounter < 3000) 
	 // {
		// while (SP->ReadData(incomingData, 3) == 3 && incomingData[2] == 65)
		// {
			// scounter ++;
			// forceIn = (decompress12bit(incomingData));
			// incomingData[readResult] = 0;
			// printf("%f\n", forceIn);

		// }
	 // }
	
	
    printf("Simulation run time %f sec, Lfinal = %f\n", realTime, lf);
	printf("Forcei = %f \n", forcei);
    //fprintf(f, "%Lf\n", arrayout[0]);
    



    // delete everything we allocated
 

    // terminate

    mj_deactivate();
    


    return 0;
}


