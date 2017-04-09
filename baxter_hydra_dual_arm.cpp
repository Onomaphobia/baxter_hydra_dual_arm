#include <iostream>
#include <iomanip>
#include <vector>
#include <time.h>
#include "matrix.h"
#include "math.h"

using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

//client header
#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <string>
// link with Ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")
#define DEFAULT_PORT "10000" 
#define DEFAULT_BUFFER_LENGTH 4096
#define PI 3.1415926
#define IP_ADDRESS "192.168.2.113"
#define LEFT_ARM_ID 0
#define RIGHT_ARM_ID 1

//sixense header and lib
#include <sixense.h>
#include <sixense_utils/derivatives.hpp>
#include <sixense_utils/button_states.hpp>
#include <sixense_utils/event_triggers.hpp>
#include <sixense_utils/controller_manager/controller_manager.hpp>




// flags that the controller manager system can set to tell the graphics system to draw the instructions
// for the player
static bool controller_manager_screen_visible = true;
std::string controller_manager_text_string;


/******************************************************************************
Client begin 
*******************************************************************************/
class Client {
public:
	Client(char* servername)
	{
		szServerName = servername;
		ConnectSocket = INVALID_SOCKET;
	}

	bool Start() {
		WSADATA wsaData;

		// Initialize Winsock
		int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
		if(iResult != 0)
		{
			printf("WSAStartup failed: %d\n", iResult);
			return false;
		}

		struct addrinfo	*result = NULL,
						*ptr = NULL,
						hints;

		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC;		
		hints.ai_socktype = SOCK_STREAM;	
		hints.ai_protocol = IPPROTO_TCP;

		// Resolve the server address and port
		iResult = getaddrinfo(szServerName, DEFAULT_PORT, &hints, &result);
		if (iResult != 0)
		{
			printf("getaddrinfo failed: %d\n", iResult);
			WSACleanup();
			return false;
		}

		ptr = result;

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (ConnectSocket == INVALID_SOCKET)
		{
			printf("Error at socket(): %d\n", WSAGetLastError());
			freeaddrinfo(result);
			WSACleanup();
			return false;
		}

		// Connect to server
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

		if (iResult == SOCKET_ERROR)
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
		}


		freeaddrinfo(result);

		if (ConnectSocket == INVALID_SOCKET)
		{
			printf("Unable to connect to server!\n");
			WSACleanup();
			return false;
		}

		return true;
	};

	// Free the resouces
	void Stop() {
		int iResult = shutdown(ConnectSocket, SD_SEND);

		if (iResult == SOCKET_ERROR)
		{
			printf("shutdown failed: %d\n", WSAGetLastError());
		}

		closesocket(ConnectSocket);
		WSACleanup();
	};

	// Send message to server
	bool Send(char* szMsg)
	{
		int iResult = send(ConnectSocket, szMsg, strlen(szMsg), 0);

		if (iResult == SOCKET_ERROR)
		{
			printf("send failed: %d\n", WSAGetLastError());
			Stop();
			return false;
		}

		return true;
	};

	// Receive message from server
	bool Recv(char* destMsg, int maxSize)
	{
		char recvbuf[DEFAULT_BUFFER_LENGTH];
		int iResult = recv(ConnectSocket, recvbuf, DEFAULT_BUFFER_LENGTH, 0);

		if (iResult > 0)
		{
			//char msg[DEFAULT_BUFFER_LENGTH];
			//char data[DEFAULT_BUFFER_LENGTH];

			memset(destMsg, 0, maxSize);
			strncpy(destMsg, recvbuf, iResult);

			return true;
		}
		return false;
	}

private:
	char* szServerName;
	SOCKET ConnectSocket;
};
/******************************************************************************
Client end here
*******************************************************************************/



/**************************************************
 This is the callback that gets registered with the sixenseUtils::controller_manager. It will get called each time the user completes
 one of the setup steps so that the game can update the instructions to the user. If the engine supports texture mapping, the 
 controller_manager can prove a pathname to a image file that contains the instructions in graphic form.
 The controller_manager serves the following functions:
  1) Makes sure the appropriate number of controllers are connected to the system. The number of required controllers is designaged by the
     game type (ie two player two controller game requires 4 controllers, one player one controller game requires one)
  2) Makes the player designate which controllers are held in which hand.
  3) Enables hemisphere tracking by calling the Sixense API call sixenseAutoEnableHemisphereTracking. After this is completed full 360 degree
     tracking is possible.
**************************************************/


void controller_manager_setup_callback(sixenseUtils::ControllerManager::setup_step step) 
{

	if (sixenseUtils::getTheControllerManager()->isMenuVisible()) 
	{

		// Turn on the flag that tells the graphics system to draw the instruction screen instead of the controller information. The game
		// should be paused at this time.
		controller_manager_screen_visible = true;

		// Ask the controller manager what the next instruction string should be.
		controller_manager_text_string = sixenseUtils::getTheControllerManager()->getStepString();

		// We could also load the supplied controllermanager textures using the filename: sixenseUtils::getTheControllerManager()->getTextureFileName();

	}
	else
	{

		// We're done with the setup, so hide the instruction screen.
		controller_manager_screen_visible = false;

	}

}

/**************************************************************

InverseKinematics

**************************************************************/


Matrix cross(Matrix a, Matrix b)
{
    Matrix r(3,1);
    r(0,0) = a(1,0)*b(2,0)-a(2,0)*b(1,0);
    r(1,0) = a(2,0)*b(0,0)-a(0,0)*b(2,0);
    r(2,0) = a(0,0)*b(1,0)-a(1,0)*b(0,0);
    return r;
}

Matrix delta(Matrix Ti, Matrix Tg)
{
	Matrix ep(3,1),eo(3,1),delta(6,1);

	int i,j;
	for (i=0; i < 3; i++)
	{
		ep(i,0) = Tg(i,3)-Ti(i,3);
	}

	Matrix temp3(3,1);
	temp3.Null(3,1);

	for (j=0; j<3; j++)
    {
		Matrix temp1(3,1), temp2(3,1);
		temp1(0,0) = (Ti(0,j));
		temp1(1,0) = (Ti(1,j));
		temp1(2,0) = (Ti(2,j));
		temp2(0,0) = (Tg(0,j));
		temp2(1,0) = (Tg(1,j));
		temp2(2,0) = (Tg(2,j));
		Matrix crossed(3,1);
		crossed=cross(temp1, temp2);
		temp3(0,0)=temp3(0,0)+crossed(0,0);
		temp3(1,0)=temp3(1,0)+crossed(1,0);
		temp3(2,0)=temp3(2,0)+crossed(2,0);
	}
	eo(0,0)=temp3(0,0)/2;
	eo(1,0)=temp3(1,0)/2;
	eo(2,0)=temp3(2,0)/2;
	//eo*=(0.5);

	// delta definition
	for (i=0;i<3;i++)
    {
		delta(i,0)=ep(i,0);
	}
	for (i=3;i<6;i++)
	{
		delta(i,0)=eo(i-3,0);
	}

////eo=0.5*( cross(Ti(1:3,1),Td(1:3,1)) + cross(Ti(1:3,2),Td(1:3,2)) + cross(Ti(1:3,3),Td(1:3,3)) );
////From equation 17 on page 189 of (Robot Motion Planning and Control) Book by Micheal Brady et al. Taken from the paper (Resolved-Acceleration Control of Mechanical Manipulators) By John Y. S. Luh et al.
	return delta;
}

Matrix jacobian(int arm_ID,double L1, double L2, double L4, double L6, double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7, double d1, double d2, double d3, double d4, double d5, double d6)
{
	if (arm_ID == LEFT_ARM_ID)
	{
		Matrix Jo(6,7);

	Jo(0,0) = -L4*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - d4*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - d3*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - d5*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - d1*sin(theta1 + PI/4) - L6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - sin(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - L2*sin(PI/2 + theta2)*sin(theta1 + PI/4) - d2*cos(PI/2 + theta2)*sin(theta1 + PI/4);
	Jo(0,1) = L4*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 + PI/4) - cos(theta3)*cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - d6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - cos(theta1 + PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 + PI/4) - cos(theta3)*cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2))) - d4*(cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - d5*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - cos(theta1 + PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - L6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - cos(theta1 + PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 + PI/4) - cos(theta3)*cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2))) + L2*cos(PI/2 + theta2)*cos(theta1 + PI/4) - d2*cos(theta1 + PI/4)*sin(PI/2 + theta2) - d3*cos(theta3)*cos(theta1 + PI/4)*sin(PI/2 + theta2);
	Jo(0,2) = L6*(sin(theta6)*(sin(theta5)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + d6*(cos(theta6)*(sin(theta5)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + sin(theta4)*sin(theta6)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - d3*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3)) + d5*(sin(theta5)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - L4*sin(theta4)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3)) - d4*cos(theta4)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3));
	Jo(0,3) = d4*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - L6*(cos(theta6)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2))) - L4*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + d6*(sin(theta6)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + cos(theta5)*cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2))) + d5*cos(theta5)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2));
	Jo(0,4) = d5*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + L6*sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + d6*cos(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3)));
	Jo(0,5) = d6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2))) - L6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)));
	Jo(0,6) = 0;

	Jo(1,0) = d1*cos(theta1 + PI/4) - d4*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - d3*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - d5*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - L4*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2)) - L6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) + cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2))) - d6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2))) + L2*cos(theta1 + PI/4)*sin(PI/2 + theta2) + d2*cos(PI/2 + theta2)*cos(theta1 + PI/4);
	Jo(1,1) = L4*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 + PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - L6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 + PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 + PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d4*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 + PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - d6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 + PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 + PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d5*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 + PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + L2*cos(PI/2 + theta2)*sin(theta1 + PI/4) - d2*sin(PI/2 + theta2)*sin(theta1 + PI/4) - d3*cos(theta3)*sin(PI/2 + theta2)*sin(theta1 + PI/4);
	Jo(1,2) = d3*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4)) - d6*(cos(theta6)*(sin(theta5)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) + sin(theta4)*sin(theta6)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - L6*(sin(theta6)*(sin(theta5)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - cos(theta6)*sin(theta4)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - d5*(sin(theta5)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) + L4*sin(theta4)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4)) + d4*cos(theta4)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4));
	Jo(1,3) = L4*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - d4*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + L6*(cos(theta6)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d6*(sin(theta6)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta5)*cos(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d5*cos(theta5)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4));
	Jo(1,4) = -d5*(sin(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - L6*sin(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - d6*cos(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4)));
	Jo(1,5) = L6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) - sin(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4))) - d6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)));
	Jo(1,6) = 0;

	Jo(2,0) = 0;
	Jo(2,1) = d5*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) - L4*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4)) + d4*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + L6*(sin(theta6)*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4))) + d6*(cos(theta6)*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) + sin(theta6)*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4))) - L2*sin(PI/2 + theta2) - d2*cos(PI/2 + theta2) - d3*cos(theta3)*cos(PI/2 + theta2);
	Jo(2,2)	= d5*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + L6*(sin(theta6)*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + cos(theta6)*sin(theta3)*sin(theta4)*sin(PI/2 + theta2)) + d6*(cos(theta6)*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta4)*sin(theta6)*sin(PI/2 + theta2)) + d3*sin(theta3)*sin(PI/2 + theta2) + L4*sin(theta3)*sin(theta4)*sin(PI/2 + theta2) + d4*cos(theta4)*sin(theta3)*sin(PI/2 + theta2);
	Jo(2,3)	= d6*(sin(theta6)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - cos(theta5)*cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - d4*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)) - L6*(cos(theta6)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - L4*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - d5*cos(theta5)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2));
	Jo(2,4)	= d5*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + L6*sin(theta6)*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + d6*cos(theta6)*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2));
	Jo(2,5)	= d6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - L6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)));
	Jo(2,6)	= 0;

    Jo(3,0) = 0;
	Jo(3,1)	= -sin(theta1 + PI/4);
	Jo(3,2)	= cos(theta1 + PI/4)*sin(PI/2 + theta2);
	Jo(3,3)	= -cos(theta3)*sin(theta1 + PI/4) - cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3);
	Jo(3,4)	= cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2) - sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4));
	Jo(3,5) = sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3));
	Jo(3,6)	= -sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) + cos(theta1 + PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 + PI/4) + cos(PI/2 + theta2)*cos(theta1 + PI/4)*sin(theta3))) - cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 + PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 + PI/4)) - cos(theta4)*cos(theta1 + PI/4)*sin(PI/2 + theta2));

	Jo(4,0) = 0;
	Jo(4,1) = cos(theta1 + PI/4);
	Jo(4,2) = sin(PI/2 + theta2)*sin(theta1 + PI/4);
	Jo(4,3) = cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4);
	Jo(4,4) = sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4);
	Jo(4,5) = cos(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4)) - sin(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4));
	Jo(4,6) = sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 + PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 + PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 + PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 + PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 + PI/4));

	Jo(5,0) = 1;
	Jo(5,1) = 0;
	Jo(5,2) = cos(PI/2 + theta2);
	Jo(5,3) = sin(theta3)*sin(PI/2 + theta2);
	Jo(5,4) = cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2);
	Jo(5,5) = sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2);
	Jo(5,6) = cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)) - sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2));

	return Jo;
	}

	if (arm_ID == RIGHT_ARM_ID)
	{
		Matrix Jo(6,7);

	Jo(0,0) = -L4*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - d4*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - d3*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - d5*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - d1*sin(theta1 - PI/4) - L6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - sin(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - L2*sin(PI/2 + theta2)*sin(theta1 - PI/4) - d2*cos(PI/2 + theta2)*sin(theta1 - PI/4);
	Jo(0,1) = L4*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 - PI/4) - cos(theta3)*cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - d6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - cos(theta1 - PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 - PI/4) - cos(theta3)*cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2))) - d4*(cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - d5*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - cos(theta1 - PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - L6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta4) + cos(theta3)*cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - cos(theta1 - PI/4)*sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2)*cos(theta1 - PI/4) - cos(theta3)*cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2))) + L2*cos(PI/2 + theta2)*cos(theta1 - PI/4) - d2*cos(theta1 - PI/4)*sin(PI/2 + theta2) - d3*cos(theta3)*cos(theta1 - PI/4)*sin(PI/2 + theta2);
	Jo(0,2) = L6*(sin(theta6)*(sin(theta5)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + d6*(cos(theta6)*(sin(theta5)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + sin(theta4)*sin(theta6)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - d3*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3)) + d5*(sin(theta5)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - L4*sin(theta4)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3)) - d4*cos(theta4)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3));
	Jo(0,3) = d4*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - L6*(cos(theta6)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2))) - L4*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + d6*(sin(theta6)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + cos(theta5)*cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2))) + d5*cos(theta5)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2));
	Jo(0,4) = d5*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + L6*sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + d6*cos(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3)));
	Jo(0,5) = d6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2))) - L6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)));
	Jo(0,6) = 0;

	Jo(1,0) = d1*cos(theta1 - PI/4) - d4*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - d3*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - d5*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - L4*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2)) - L6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) + cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2))) - d6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - sin(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2))) + L2*cos(theta1 - PI/4)*sin(PI/2 + theta2) + d2*cos(PI/2 + theta2)*cos(theta1 - PI/4);
	Jo(1,1) = L4*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 - PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - L6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 - PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 - PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d4*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 - PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - d6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 - PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2)*sin(theta1 - PI/4) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d5*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4)*sin(theta1 - PI/4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + L2*cos(PI/2 + theta2)*sin(theta1 - PI/4) - d2*sin(PI/2 + theta2)*sin(theta1 - PI/4) - d3*cos(theta3)*sin(PI/2 + theta2)*sin(theta1 - PI/4);
	Jo(1,2) = d3*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4)) - d6*(cos(theta6)*(sin(theta5)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) + sin(theta4)*sin(theta6)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - L6*(sin(theta6)*(sin(theta5)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - cos(theta6)*sin(theta4)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - d5*(sin(theta5)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta4)*cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) + L4*sin(theta4)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4)) + d4*cos(theta4)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4));
	Jo(1,3) = L4*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - d4*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + L6*(cos(theta6)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d6*(sin(theta6)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta5)*cos(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d5*cos(theta5)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4));
	Jo(1,4) = -d5*(sin(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - L6*sin(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - d6*cos(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) - cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4)));
	Jo(1,5) = L6*(cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) - sin(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4))) - d6*(sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)));
	Jo(1,6) = 0;

	Jo(2,0) = 0;
	Jo(2,1) = d5*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) - L4*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4)) + d4*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + L6*(sin(theta6)*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4))) + d6*(cos(theta6)*(cos(theta5)*(sin(theta4)*sin(PI/2 + theta2) - cos(theta3)*cos(theta4)*cos(PI/2 + theta2)) + cos(PI/2 + theta2)*sin(theta3)*sin(theta5)) + sin(theta6)*(cos(theta4)*sin(PI/2 + theta2) + cos(theta3)*cos(PI/2 + theta2)*sin(theta4))) - L2*sin(PI/2 + theta2) - d2*cos(PI/2 + theta2) - d3*cos(theta3)*cos(PI/2 + theta2);
	Jo(2,2)	= d5*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + L6*(sin(theta6)*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + cos(theta6)*sin(theta3)*sin(theta4)*sin(PI/2 + theta2)) + d6*(cos(theta6)*(cos(theta3)*sin(theta5)*sin(PI/2 + theta2) + cos(theta4)*cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta4)*sin(theta6)*sin(PI/2 + theta2)) + d3*sin(theta3)*sin(PI/2 + theta2) + L4*sin(theta3)*sin(theta4)*sin(PI/2 + theta2) + d4*cos(theta4)*sin(theta3)*sin(PI/2 + theta2);
	Jo(2,3)	= d6*(sin(theta6)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - cos(theta5)*cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - d4*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)) - L6*(cos(theta6)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - L4*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - d5*cos(theta5)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2));
	Jo(2,4)	= d5*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + L6*sin(theta6)*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2)) + d6*cos(theta6)*(sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2));
	Jo(2,5)	= d6*(sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) - cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2))) - L6*(cos(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2)) + sin(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)));
	Jo(2,6)	= 0;

    Jo(3,0) = 0;
	Jo(3,1)	= -sin(theta1 - PI/4);
	Jo(3,2)	= cos(theta1 - PI/4)*sin(PI/2 + theta2);
	Jo(3,3)	= -cos(theta3)*sin(theta1 - PI/4) - cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3);
	Jo(3,4)	= cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2) - sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4));
	Jo(3,5) = sin(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) - cos(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3));
	Jo(3,6)	= -sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) + cos(theta1 - PI/4)*sin(theta4)*sin(PI/2 + theta2)) + sin(theta5)*(cos(theta3)*sin(theta1 - PI/4) + cos(PI/2 + theta2)*cos(theta1 - PI/4)*sin(theta3))) - cos(theta6)*(sin(theta4)*(sin(theta3)*sin(theta1 - PI/4) - cos(theta3)*cos(PI/2 + theta2)*cos(theta1 - PI/4)) - cos(theta4)*cos(theta1 - PI/4)*sin(PI/2 + theta2));

	Jo(4,0) = 0;
	Jo(4,1) = cos(theta1 - PI/4);
	Jo(4,2) = sin(PI/2 + theta2)*sin(theta1 - PI/4);
	Jo(4,3) = cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4);
	Jo(4,4) = sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4);
	Jo(4,5) = cos(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4)) - sin(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4));
	Jo(4,6) = sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) - sin(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4)) + sin(theta5)*(cos(theta3)*cos(theta1 - PI/4) - cos(PI/2 + theta2)*sin(theta3)*sin(theta1 - PI/4))) + cos(theta6)*(sin(theta4)*(cos(theta1 - PI/4)*sin(theta3) + cos(theta3)*cos(PI/2 + theta2)*sin(theta1 - PI/4)) + cos(theta4)*sin(PI/2 + theta2)*sin(theta1 - PI/4));

	Jo(5,0) = 1;
	Jo(5,1) = 0;
	Jo(5,2) = cos(PI/2 + theta2);
	Jo(5,3) = sin(theta3)*sin(PI/2 + theta2);
	Jo(5,4) = cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2);
	Jo(5,5) = sin(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) + cos(theta5)*sin(theta3)*sin(PI/2 + theta2);
	Jo(5,6) = cos(theta6)*(cos(theta4)*cos(PI/2 + theta2) - cos(theta3)*sin(theta4)*sin(PI/2 + theta2)) - sin(theta6)*(cos(theta5)*(cos(PI/2 + theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(PI/2 + theta2)) - sin(theta3)*sin(theta5)*sin(PI/2 + theta2));

	return Jo;
	}
	
}

Matrix trans(double alpha, double a, double d, double theta)
{
	Matrix T(4,4);

	double s, c, sa, ca;
	s = sin(theta);
	c = cos(theta);
	sa = sin(alpha);
	ca = cos(alpha);
	T(0,0) = c;
	T(0,1) = -s;
	T(0,2) = 0;
	T(0,3) = a;
	T(1,0) = s*ca;
	T(1,1) = c*ca;
	T(1,2) = -sa;
	T(1,3) = -sa*d;
	T(2,0) = s*sa;
	T(2,1) = c*sa;
	T(2,2) = ca;
	T(2,3) = ca*d;
	T(3,0) = 0;
	T(3,1) = 0;
	T(3,2) = 0;
	T(3,3) = 1;
	return T;
}

Matrix kinematics(int arm_ID, double L1, double L2, double L4, double L6, double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7, double d1, double d2, double d3, double d4, double d5, double d6)
{
	if (arm_ID == LEFT_ARM_ID)
	{
		Matrix Tb0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4), T07(4,4);
	
		Tb0(0,0) = 1;
		Tb0(0,1) = 0;
		Tb0(0,2) = 0;
		Tb0(0,3) = -6.4027;
		Tb0(1,0) = 0;
		Tb0(1,1) = 1;
		Tb0(1,2) = 0;
		Tb0(1,3) = 25.9027;
		Tb0(2,0) = 0;
		Tb0(2,1) = 0;
		Tb0(2,2) = 1;
		Tb0(2,3) = 12.9626;
		Tb0(3,0) = 0;
		Tb0(3,1) = 0;
		Tb0(3,2) = 0;
		Tb0(3,3) = 1;

		T01 = trans(  0, 0,L1,theta1+PI/4);
		T12 = trans(-PI/2,d1, 0,theta2+PI/2);
		T23 = trans( PI/2,d2,L2,theta3);
		T34 = trans(-PI/2,d3, 0,theta4);
		T45 = trans( PI/2,d4,L4,theta5);
		T56 = trans(-PI/2,d5, 0,theta6);
		T67 = trans( PI/2,d6,L6,theta7);

		T07 = Tb0*T01*T12*T23*T34*T45*T56*T67;
		return T07;
	}
	
	if (arm_ID == RIGHT_ARM_ID)
	{
		Matrix Tb0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4), T07(4,4);
	
		Tb0(0,0) = 1;
		Tb0(0,1) = 0;
		Tb0(0,2) = 0;
		Tb0(0,3) = -6.4027;
		Tb0(1,0) = 0;
		Tb0(1,1) = 1;
		Tb0(1,2) = 0;
		Tb0(1,3) = -25.9027;
		Tb0(2,0) = 0;
		Tb0(2,1) = 0;
		Tb0(2,2) = 1;
		Tb0(2,3) = 12.9626;
		Tb0(3,0) = 0;
		Tb0(3,1) = 0;
		Tb0(3,2) = 0;
		Tb0(3,3) = 1;

		T01 = trans(  0, 0,L1,theta1-PI/4);
		T12 = trans(-PI/2,d1, 0,theta2+PI/2);
		T23 = trans( PI/2,d2,L2,theta3);
		T34 = trans(-PI/2,d3, 0,theta4);
		T45 = trans( PI/2,d4,L4,theta5);
		T56 = trans(-PI/2,d5, 0,theta6);
		T67 = trans( PI/2,d6,L6,theta7);

		T07 = Tb0*T01*T12*T23*T34*T45*T56*T67;
		return T07;
	}
}

Matrix TM_goal(double x, double y, double z, Matrix rotation)
{
	Matrix Tg(4,4);

	Tg(0,0) = rotation(0,0);
	Tg(0,1) = rotation(0,1);
	Tg(0,2) = rotation(0,2);
	Tg(0,3) = x;

	Tg(1,0) = rotation(1,0);
	Tg(1,1) = rotation(1,1);
	Tg(1,2) = rotation(1,2);
	Tg(1,3) = y;

	Tg(2,0) = rotation(2,0);
	Tg(2,1) = rotation(2,1);
	Tg(2,2) = rotation(2,2);
	Tg(2,3) = z;

	Tg(3,0) = 0;
	Tg(3,1) = 0;
	Tg(3,2) = 0;
	Tg(3,3) = 1;

	return Tg;
}

//From hydra rotation to arm rotation
Matrix Rot_Map(Matrix Hydra_rot)
{
	Matrix hydra2arm_rot(3,3), hydra_cali(3,3);
	Matrix Temp1(3,3), Temp2(3,3);

	hydra2arm_rot(0,0) = 0;
	hydra2arm_rot(0,1) = 0;
	hydra2arm_rot(0,2) = -1;
	hydra2arm_rot(1,0) = -1;
	hydra2arm_rot(1,1) = 0;
	hydra2arm_rot(1,2) = 0;
	hydra2arm_rot(2,0) = 0;
	hydra2arm_rot(2,1) = 1;
	hydra2arm_rot(2,2) = 0;

	hydra_cali(0,0) = 0;
	hydra_cali(0,1) = -1;
	hydra_cali(0,2) = 0;
	hydra_cali(1,0) = -1;
	hydra_cali(1,1) = 0;
	hydra_cali(1,2) = 0;
	hydra_cali(2,0) = 0;
	hydra_cali(2,1) = 0;
	hydra_cali(2,2) = -1;

	Temp1 = hydra2arm_rot * Hydra_rot;
	Temp2 = Temp1*hydra_cali;

	return Temp2;
}

Matrix inversekinematics(int arm_ID, double goal[], Matrix rotation, double theta[], double velocity, double wo, double ko, double& _x7, double& _y7, double& _z7, double& _alpha, double& _beta, double& _gamma, double& _mm, vector<double>& joint)
{
    // Link length (cm)
	double L1=23;
    double L2=36.44;
    double L4=37.429;
    double L6=22.9525; //11.6;
    double d1=6.9;
    double d2=0;
    double d3=6.9;
    double d4=0;
    double d5=0;
    double d6=1;

	double a1;
	double a2;
	double a3;
	double a4;
	double a5;
	double a6;
	double a7;

	/*
	goal[0] = 60;
	goal[1] = -33;
	goal[2] = 40;
	goal[3] = 0;
	goal[4] = 0;
	goal[5] = PI/2;
	*/
	
	a1 = theta[0];
	a2 = theta[1];
	a3 = theta[2];
	a4 = theta[3];
	a5 = theta[4];
	a6 = theta[5];
	a7 = theta[6];


	double xg, yg, zg;

	Matrix Tg(4,4), T07(4,4), R07(3,3);
	//double xg, yg, zg
	//double xi, yi, zi, ai, Bi, ri;
	//double D, t, dt, time, n;
	//double Xdot, Ydot, Zdot, adot, Bdot, rdot;
	double da1, da2, da3, da4, da5, da6, da7;
		
	double sf;
	Matrix J(6,7), Jtrans(7,6), Jtemp(6,6);
	double detJ;
	Matrix dq(7,1);
	Matrix dq_check(7,1);
	
	//Ti07 = kinematics(L1, L2, L4, L6, a1i, a2i, a3i, a4i, a5i, a6i, a7i, d1, d2, d3, d4, d5, d6);
	T07 = kinematics(arm_ID, L1, L2, L4, L6, a1, a2, a3, a4, a5, a6, a7, d1, d2, d3, d4, d5, d6);

	
	
	//rotation = Hydra_rot*R07;

	if (fabs(T07(0,0))<0.0001 && fabs(T07(1,0))<0.0001)
		{
			_alpha = 0;
			_beta = (atan2(-T07(2,0), T07(0,0)))*180/3.1415926;
			_gamma = (atan2(-T07(1,2), T07(1,1)))*180/3.1415926;
		}
		else
		{
			_alpha = atan2( T07(1,0), T07(0,0));
			double s = sin(_alpha);
			double c = cos(_alpha);
			_beta = (atan2(-T07(2,0), c*T07(0,0)+s*T07(1,0)))*180/3.1415926;
			_gamma = (atan2(s*T07(0,2)-c*T07(1,2), c*T07(1,1)-s*T07(0,1)))*180/3.1415926;
		}
	_alpha = _alpha*180/3.1415926;


	_x7 = T07(0,3);
	_y7 = T07(1,3);
	_z7 = T07(2,3);
	

	xg = goal[0];
	yg = goal[1];
	zg = goal[2];

	//ag = goal[3];
	//Bg = goal[4];
	//rg = goal[5];
	

	Tg = TM_goal(xg, yg, zg, rotation);


	//Tg = TM_goal(xg, yg, zg, ag, Bg, rg);


	/*
	if (fabs(xg - _x7_right)>1 || fabs(yg -_y7_right)>1 || fabs(zg - _z7_right)>1)
	{
		xg = _x7_right + (xg - _x7_right)*1/sqrt((xg - _x7_right)*(xg - _x7_right)+(yg - _y7_right)*(yg - _y7_right)+(zg - _z7_right)*(zg - _z7_right));
		yg = _y7_right + (yg - _y7_right)*1/sqrt((xg - _x7_right)*(xg - _x7_right)+(yg - _y7_right)*(yg - _y7_right)+(zg - _z7_right)*(zg - _z7_right));
		zg = _z7_right + (zg - _z7_right)*1/sqrt((xg - _x7_right)*(xg - _x7_right)+(yg - _y7_right)*(yg - _y7_right)+(zg - _z7_right)*(zg - _z7_right));
	}
	*/

	//
	

	Matrix q(7,1);

	q(0,0) = a1;
	q(1,0) = a2;
	q(2,0) = a3;
	q(3,0) = a4;
	q(4,0) = a5;
	q(5,0) = a6;
	q(6,0) = a7;


	Matrix dH(7,1), dHo(7,1), qmin(7,1), qmax(7,1), qmid(7,1), W(7,7), Winv(7,7), diag(7,1), pinvJsr(7,7);

	//dHo(0,0) = Hmatrix[0];
	//dHo(1,0) = Hmatrix[1];
	//dHo(2,0) = Hmatrix[2];
	//dHo(3,0) = Hmatrix[3];
	//dHo(4,0) = Hmatrix[4];
	//dHo(5,0) = Hmatrix[5];
	//dHo(6,0) = Hmatrix[6];
	
	qmax(0,0) = 1.7017;
	qmax(1,0) = 1.047;
	qmax(2,0) = 3.0542;
	qmax(3,0) = 2.618;
	qmax(4,0) = 3.059;
	qmax(5,0) = 2.094;
	qmax(6,0) = 3.059;

	qmin(0,0) = -1.7017;
	qmin(1,0) = -2.147;
	qmin(2,0) = -3.0542;
	qmin(3,0) = -0.052;
	qmin(4,0) = -3.059;
	qmin(5,0) = -1.571;
	qmin(6,0) = -3.059;

	for (int i = 0; i < 7; i++)
	{
		qmid(i,0) = (qmin(i,0)+qmax(i,0))/2;
	}
	/*
	qmin(0,0) = -1.7017;
	qmin(1,0) = -1.047;
	qmin(2,0) = -3.0542;
	qmin(3,0) = -2.618;
	qmin(4,0) = -3.059;
	qmin(5,0) = -2.094;
	qmin(6,0) = -3.059;

	qmax(0,0) = 1.7017;
	qmax(1,0) = 2.147;
	qmax(2,0) = 3.0542;
	qmax(3,0) = 0.05;
	qmax(4,0) = 3.059;
	qmax(5,0) = 1.5708;
	qmax(6,0) = 3.059;
	*/


	//double inf = std::numeric_limits<double>::infinity();
	Matrix dx(6,1);

	// Calculate Jacobian and its determinant by using jacobian function.

	J= jacobian(arm_ID,L1,L2,L4,L6,a1,a2,a3,a4,a5,a6,a7,d1,d2,d3,d4,d5,d6);

	dx=delta(T07,Tg); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	Jtrans = ~J;

	//dH as criteria of joint limits
	for (int j=0; j<7; j++)
	{
		dH(j,0)=-0.25*(qmax(j,0)-qmin(j,0))*(qmax(j,0)-qmin(j,0))*(2*q(j,0)-qmax(j,0)-qmin(j,0))/((qmax(j,0)-q(j,0))*(qmax(j,0)-q(j,0))*(q(j,0)-qmin(j,0))*(q(j,0)-qmin(j,0))); // condition1
		// Re-defining the weight in case the joint is moving away from it's limit or the joint limit was exceeded:

//        if (abs(dH(j,0)) < abs(dHo(j,0)) && q(j,0) < qmax(j,0) && q(j,0) > qmin(j,0)) // condition2
//            dH(j,0)=0;
//        else if (abs(dH(j,0)) <= abs(dHo(j,0)) && (q(j,0) >= qmax(j,0) || q(j,0) <= qmin(j,0))) // condition3
//            dH(j,0)=9999999999;
//        else if (abs(dH(j,0)) > abs(dHo(j,0)) && (q(j,0) >= qmax(j,0) || q(j,0) <= qmin(j,0))) // condition4
//            dH(j,0)=0;
//        dHo(j,0) = dHoo(j,0);

		/*
		if (dH(j,0) > 30000)
		{
			dH(j,0) = 30000;
		}
		if (dH(j,0) < -30000)
		{
			dH(j,0) = -30000;
		}
		*/
			
	}

	//H1 = dH(0,0);
	//H2 = dH(1,0);
	//H3 = dH(2,0);
	//H4 = dH(3,0);
	//H5 = dH(4,0);
	//H6 = dH(5,0);
	//H7 = dH(6,0);


	/*****************least weighted norm begins************************/
	/*

	W.Null(7,7);
	Winv.Null(7,7);
	for (int j=0; j < 7; j++)
	{
		for (int k=0; k < 7; k++)
		{
			if (j==k)
			{
				//W(j,k)=1;
				W(j,k)=1+abs(dH(j,0));
				diag(j,0)=W(j,k);
				// The inverse of the diagonal weight matrix:
				Winv(j,k)=1/(diag(j,0));
			}
		}
	}

	// Winv.Unit(7); // switch of LWN

	Jtemp = J * Winv * Jtrans;
	//Jtemp = J * Jtrans;
	detJ = sqrt(Jtemp.Det());


	if (detJ<wo)
	{
		sf=ko*pow((1-detJ/wo),2);
	}
	else
	{
		sf = 0;
	}

	Matrix sfm(6,6);
	sfm.Unit(6);

	pinvJsr = Winv*Jtrans*!(J*Winv*Jtrans+sf*sfm); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// calculating the joint angle change optimized based on the Weighted Least Norm Solution:
	// Here, dq is the angular velocity of every joint
	dq = pinvJsr*dx;

	*/
	/*************least weighted norm ends*******************/



	/********************task priority begins***************/

	Jtemp = J * Jtrans;
	detJ = sqrt(Jtemp.Det());

	if (detJ<wo)
	{
		sf=ko*pow((1-detJ/wo),2);
	}
	else
	{
		sf = 0;
	}

	Matrix sfm(6,6);
	sfm.Unit(6);
	Matrix eye(7,7);
	eye.Unit(7);

	// calculating the SR-Inverse of the Jacobian:
	pinvJsr = Jtrans * !(J * Jtrans + sf * sfm); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// calculating the joint angle change optimized based on the Weighted Least Norm Solution:
	// Here, dq is the angular velocity of every joint
	dq = pinvJsr * dx + (eye - pinvJsr * J) * dH * 0.4;

	/*********************tast priority ends*****************/




	/************** check the direction of dq and adjust dH*******************/

	/*
	dq_check = dq;
	for (int i = 0; i<7; i++)
	{
		if ((q(i,0) < qmid(i,0) && dq_check(i,0) > 0) || (q(i,0) > qmid(i,0) && dq_check(i,0) < 0))
		{
			dH(i,0) = dH(i,0)/10000;
		}
	}

	W.Null(7,7);
	Winv.Null(7,7);
	for (int j=0; j < 7; j++)
	{
		for (int k=0; k < 7; k++)
		{
			if (j==k)
			{
				//W(j,k)=1;
				W(j,k)=1+abs(dH(j,0));
				diag(j,0)=W(j,k);
				// The inverse of the diagonal weight matrix:
				Winv(j,k)=1/(diag(j,0));
			}
		}
	}
	Jtrans = ~J;

	// Winv.Unit(7); // switch of LWN
	
	Jtemp = J * Winv * Jtrans;
	detJ = sqrt(Jtemp.Det());
	_mm = detJ;

	if (detJ<wo)
	{
		sf=ko*pow((1-detJ/wo),2);
	}
	else
	{
		sf = 0;
	}

	sfm.Unit(6);
	// calculating the SR-Inverse of the Jacobian:
	pinvJsr = Winv*Jtrans*!(J*Winv*Jtrans+sf*sfm); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// calculating the joint angle change optimized based on the Weighted Least Norm Solution:
	// Here, dq is the angular velocity of every joint
	dq = pinvJsr*dx;

	*/
	/************** check the direction of dq and adjust dH*******************/

	// Find the difference between current and next joint angles
	da1 = dq(0,0); // [rad/sec]
	da2 = dq(1,0); // [rad/sec]
	da3 = dq(2,0); // [rad/sec]
	da4 = dq(3,0); // [rad/sec]
	da5 = dq(4,0); // [rad/sec]
	da6 = dq(5,0); // [rad/sec]
	da7 = dq(6,0); // [rad/sec]



	// linear restriction
	double res;
	res = fabs(dq(0,0));
	for (int i = 1; i<7; i++)
	{
		if (res<=fabs(dq(i,0)))
		{
			res = fabs(dq(i,0));
		}
	}
	if (res > velocity)
	{
		da1 = da1*velocity/res;
		da2 = da2*velocity/res;
		da3 = da3*velocity/res;
		da4 = da4*velocity/res;
		da5 = da5*velocity/res;
		da6 = da6*velocity/res;
		da7 = da7*velocity/res;
	}

	//safety condition

	double ta1;
	double ta2;
	double ta3;
	double ta4;
	double ta5;
	double ta6;
	double ta7;

	ta1 = a1+da1;
	ta2 = a2+da2;
	ta3 = a3+da3;
	ta4 = a4+da4;
	ta5 = a5+da5;
	ta6 = a6+da6;
	ta7 = a7+da7;

	if (ta1 < qmin(0,0) || ta1 > qmax(0,0))
	{
		da1 = 0;
	}
	if (ta2 < qmin(1,0) || ta2 > qmax(1,0))
	{
		da2 = 0;
	}
	if (ta3 < qmin(2,0) || ta3 > qmax(2,0))
	{
		da3 = 0;
	}
	if (ta4 < qmin(3,0) || ta4 > qmax(3,0))
	{
		da4 = 0;
	}
	if (ta5 < qmin(4,0) || ta5 > qmax(4,0))
	{
		da5 = 0;
	}
	if (ta6 < qmin(5,0) || ta6 > qmax(5,0))
	{
		da6 = 0;
	}
	if (ta7 < qmin(6,0) || ta7 > qmax(6,0))
	{
		da7 = 0;
	}


	//Update the joint angles of the robot ( keep the units in mind, degrees or radians)

	joint[0] = a1+da1;
	joint[1] = a2+da2;
	joint[2] = a3+da3;
	joint[3] = a4+da4;
	joint[4] = a5+da5;
	joint[5] = a6+da6;
	joint[6] = a7+da7;

	T07 = kinematics(arm_ID, L1, L2, L4, L6, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], joint[6], d1, d2, d3, d4, d5, d6);

	R07(0,0) = T07(0,0);
	R07(0,1) = T07(0,1);
	R07(0,2) = T07(0,2);
	R07(1,0) = T07(1,0);
	R07(1,1) = T07(1,1);
	R07(1,2) = T07(1,2);
	R07(2,0) = T07(2,0);
	R07(2,1) = T07(2,1);
	R07(2,2) = T07(2,2);

	return R07;

}

/***********************************************************************

InverseKinematics End

***********************************************************************/

int main()
{
	
	// Init sixense
	sixenseInit();

	// Init the controller manager. This makes sure the controllers are present, assigned to left and right hands, and that
	// the hemisphere calibration is complete.
	sixenseUtils::getTheControllerManager()->setGameType(sixenseUtils::ControllerManager::ONE_PLAYER_TWO_CONTROLLER);
	sixenseUtils::getTheControllerManager()->registerSetupCallback(controller_manager_setup_callback);

	// update the controller manager with the latest controller data here
	sixenseSetActiveBase(0);
	sixenseAllControllerData acd;

	int left_arm_ID = LEFT_ARM_ID;
	int right_arm_ID = RIGHT_ARM_ID;

	int pred_key_left = 0;
	int curr_key_left = 0;
	bool vmode_SW_left = 0;
	float pred_trigger_left = 0;
	double trigger_x_left;
	double trigger_y_left;
	double trigger_z_left;
	double trigger_arm_x_left;
	double trigger_arm_y_left;
	double trigger_arm_z_left;


	int pred_key_right = 0;
	int curr_key_right = 0;
	bool vmode_SW_right = 0;
	float pred_trigger_right = 0;
	double trigger_x_right;
	double trigger_y_right;
	double trigger_z_right;
	double trigger_arm_x_right;
	double trigger_arm_y_right;
	double trigger_arm_z_right;

	//setup control parameter

	double _velocity = 0.4;				//////////////////////////////parameter
	double _ko = 250;
	double _wo = 5000;
	double _mm;



	//left arm
	double _x7_left;
	double _y7_left;
	double _z7_left;
	double _alpha_left;
	double _beta_left;
	double _gamma_left;

	double j1_left;
	double j2_left;
	double j3_left;
	double j4_left;
	double j5_left;
	double j6_left;
	double j7_left;

	double theta_left[7] = {};

	double goal_left[6] = {};

	vector<double> joint_left;

	joint_left.resize(7,0);

	int trigger_down_left = 0;

	Matrix hydra_left(3,3), pred_rot_left(3,3), curr_rot_left(3,3), curr_arm_left(3,3), trig_rot_left(3,3), trig_arm_left(3,3), arm_rot_left(3,3);

	trig_rot_left.Unit(3);
	curr_rot_left.Unit(3);
	pred_rot_left.Unit(3);
	trig_arm_left.Unit(3);
	curr_arm_left.Unit(3);
	

	//right arm
	double _x7_right;
	double _y7_right;
	double _z7_right;
	double _alpha_right;
	double _beta_right;
	double _gamma_right;

	double j1_right;
	double j2_right;
	double j3_right;
	double j4_right;
	double j5_right;
	double j6_right;
	double j7_right;

	double theta_right[7] = {};

	double goal_right[6] = {};

	vector<double> joint_right;

	joint_right.resize(7,0);

	int trigger_down_right = 0;



	//dual_arm control setup -----------------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------------------

	int dual_arm_right = 0;
	Matrix vector(3,1), ini_pos_left(3,1), ini_pos_right(3,1), final_pos_left(3,1), final_pos_right(3,1);
	vector.Null(3,1);
	ini_pos_left.Null(3,1);
	ini_pos_right.Null(3,1);
	final_pos_left.Null(3,1);
	final_pos_right.Null(3,1);

	Matrix hydra_right(3,3), pred_rot_right(3,3), curr_rot_right(3,3), curr_arm_right(3,3), trig_rot_right(3,3), trig_arm_right(3,3), arm_rot_right(3,3);

	trig_rot_right.Unit(3);
	curr_rot_right.Unit(3);
	pred_rot_right.Unit(3);
	trig_arm_right.Unit(3);
	curr_arm_right.Unit(3);
	
	
	////////////////////////////////////////////////////////////////////
	/////////////////////////////CLIENT/////////////////////////////////
	////////////////////////////////////////////////////////////////////

	string lPstr0;
	string lPstr1;
	string lPstr2;
	string lPstr3;
	string lPstr4;
	string lPstr5;
	string lPstr6;
	string lPstr7;
	string lPstr8;

	string rPstr0;
	string rPstr1;
	string rPstr2;
	string rPstr3;
	string rPstr4;
	string rPstr5;
	string rPstr6;
	string rPstr7;
	string rPstr8;

	string space;
	string msg;

	char* recvMsg = new char[DEFAULT_BUFFER_LENGTH];

	Client client(IP_ADDRESS);//<<===========================IP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	if (!client.Start())
		return 1;


//	msg="0,0,0,0,0,0,0";
		
//	client.Send((char*)msg.c_str());
	//cout << "||Sending:"<<msg<<"\n";
//	client.Recv(recvMsg, DEFAULT_BUFFER_LENGTH);
	//cout <<"Received:"<<recvMsg<<"\n";

	//sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_left[0],&theta_left[1],&theta_left[2],&theta_left[3],&theta_left[4],&theta_left[5],&theta_left[6]);

	//sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_right[0],&theta_right[1],&theta_right[2],&theta_right[3],&theta_right[4],&theta_right[5],&theta_right[6]);

	sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_left[0],&theta_left[1],&theta_left[2],&theta_left[3],&theta_left[4],&theta_left[5],&theta_left[6],&theta_right[0],&theta_right[1],&theta_right[2],&theta_right[3],&theta_right[4],&theta_right[5],&theta_right[6]);
	
	
	//initialize
	sixenseGetAllNewestData(&acd);
	sixenseUtils::getTheControllerManager()->update(&acd);

	goal_left[0] = (-(double)acd.controllers[0].pos[2]/10 + 100)*0.6;
	goal_left[1] = -(double)acd.controllers[0].pos[0]/10*0.6;
	goal_left[2] = ((double)acd.controllers[0].pos[1]/10+50)*0.6;
	


	goal_right[0] = (-(double)acd.controllers[1].pos[2]/10 + 100)*0.6;
	goal_right[1] = -(double)acd.controllers[1].pos[0]/10*0.6;
	goal_right[2] = ((double)acd.controllers[1].pos[1]/10+50)*0.6;
	

	arm_rot_left = inversekinematics(left_arm_ID, goal_left, curr_arm_left, theta_left, _velocity, _wo, _ko, _x7_left, _y7_left, _z7_left, _alpha_left, _beta_left, _gamma_left, _mm, joint_left);
	arm_rot_right = inversekinematics(right_arm_ID, goal_right, curr_arm_right, theta_right, _velocity, _wo, _ko, _x7_right, _y7_right, _z7_right, _alpha_right, _beta_right, _gamma_right, _mm, joint_right);

	//-----------------------main loop---------------------------//
	while(true)
	{	
		//update sixense data
		sixenseGetAllNewestData(&acd);
		sixenseUtils::getTheControllerManager()->update(&acd);


		//control button

		//record trigger down action
		float trigger_left = acd.controllers[0].trigger;
		if (trigger_left != 0)
			{
				trigger_down_left = 99;
			}
			else
			{
				trigger_down_left = 0;
			}

		//record gripper button
		int gripper_left = acd.controllers[0].buttons;
		

		float trigger_right = acd.controllers[1].trigger;
		if (trigger_right != 0)
			{
				trigger_down_right = 99;
			}
			else
			{
				trigger_down_right = 0;
			}

		int gripper_right = acd.controllers[1].buttons;
		
		
		//rotation matrix mapping
		hydra_left(0,0) = acd.controllers[0].rot_mat[0][0];
		hydra_left(0,1) = acd.controllers[0].rot_mat[1][0];
		hydra_left(0,2) = acd.controllers[0].rot_mat[2][0];
		hydra_left(1,0) = acd.controllers[0].rot_mat[0][1];
		hydra_left(1,1) = acd.controllers[0].rot_mat[1][1];
		hydra_left(1,2) = acd.controllers[0].rot_mat[2][1];
		hydra_left(2,0) = acd.controllers[0].rot_mat[0][2];
		hydra_left(2,1) = acd.controllers[0].rot_mat[1][2];
		hydra_left(2,2) = acd.controllers[0].rot_mat[2][2];
		curr_rot_left = Rot_Map(hydra_left);


		hydra_right(0,0) = acd.controllers[1].rot_mat[0][0];
		hydra_right(0,1) = acd.controllers[1].rot_mat[1][0];
		hydra_right(0,2) = acd.controllers[1].rot_mat[2][0];
		hydra_right(1,0) = acd.controllers[1].rot_mat[0][1];
		hydra_right(1,1) = acd.controllers[1].rot_mat[1][1];
		hydra_right(1,2) = acd.controllers[1].rot_mat[2][1];
		hydra_right(2,0) = acd.controllers[1].rot_mat[0][2];
		hydra_right(2,1) = acd.controllers[1].rot_mat[1][2];
		hydra_right(2,2) = acd.controllers[1].rot_mat[2][2];
		curr_rot_right = Rot_Map(hydra_right);


		//record the position & orientation when the trigger is down
		if (pred_trigger_left == 0 && acd.controllers[0].trigger != 0)
		{
			trigger_x_left = (-(double)acd.controllers[0].pos[2]/10 + 100)*0.6;
			trigger_y_left = -(double)acd.controllers[0].pos[0]/10*0.6;
			trigger_z_left = ((double)acd.controllers[0].pos[1]/10+50)*0.6;
			trigger_arm_x_left = _x7_left;
			trigger_arm_y_left = _y7_left;
			trigger_arm_z_left = _z7_left;
			
			trig_rot_left(0,0) = acd.controllers[0].rot_mat[0][0];
			trig_rot_left(0,1) = acd.controllers[0].rot_mat[1][0];
			trig_rot_left(0,2) = acd.controllers[0].rot_mat[2][0];
			trig_rot_left(1,0) = acd.controllers[0].rot_mat[0][1];
			trig_rot_left(1,1) = acd.controllers[0].rot_mat[1][1];
			trig_rot_left(1,2) = acd.controllers[0].rot_mat[2][1];
			trig_rot_left(2,0) = acd.controllers[0].rot_mat[0][2];
			trig_rot_left(2,1) = acd.controllers[0].rot_mat[1][2];
			trig_rot_left(2,2) = acd.controllers[0].rot_mat[2][2];
			trig_rot_left = Rot_Map(trig_rot_left);
			curr_rot_left = trig_rot_left;
			trig_arm_left = arm_rot_left;
		}
		pred_trigger_left = acd.controllers[0].trigger;



		if (pred_trigger_right == 0 && acd.controllers[1].trigger != 0)
		{
			trigger_x_right = (-(double)acd.controllers[1].pos[2]/10 + 100)*0.6;
			trigger_y_right = -(double)acd.controllers[1].pos[0]/10*0.6;
			trigger_z_right = ((double)acd.controllers[1].pos[1]/10+50)*0.6;
			trigger_arm_x_right = _x7_right;
			trigger_arm_y_right = _y7_right;
			trigger_arm_z_right = _z7_right;
			
			hydra_right(0,0) = acd.controllers[1].rot_mat[0][0];
			hydra_right(0,1) = acd.controllers[1].rot_mat[1][0];
			hydra_right(0,2) = acd.controllers[1].rot_mat[2][0];
			hydra_right(1,0) = acd.controllers[1].rot_mat[0][1];
			hydra_right(1,1) = acd.controllers[1].rot_mat[1][1];
			hydra_right(1,2) = acd.controllers[1].rot_mat[2][1];
			hydra_right(2,0) = acd.controllers[1].rot_mat[0][2];
			hydra_right(2,1) = acd.controllers[1].rot_mat[1][2];
			hydra_right(2,2) = acd.controllers[1].rot_mat[2][2];
			trig_rot_right = Rot_Map(hydra_right);
			curr_rot_right = trig_rot_right;
			trig_arm_right = arm_rot_right;
		}
		pred_trigger_right = acd.controllers[1].trigger;

		pred_key_left = curr_key_left;
		curr_key_left = acd.controllers[0].buttons;
		
		//mode switch
		/*
		if (pred_key_left == 0 && curr_key_left == 16)
		{
			vmode_SW_left = true;
			cout<<"left velocity control"<<endl;
		}
		if (pred_key_left == 0 && curr_key_left == 64)
		{
			vmode_SW_left = false;
			cout<<"left position control"<<endl;
		}
		if (vmode_SW_left)// velocity control //
		{
			goal_left[0] = ((-(double)acd.controllers[0].pos[2]/10 + 100)*0.6 - trigger_x_left)*0.3 + _x7_left;
			goal_left[1] = (-(double)acd.controllers[0].pos[0]/10*0.6 - trigger_y_left)*0.3 + _y7_left;
			goal_left[2] = (((double)acd.controllers[0].pos[1]/10+50)*0.6 - trigger_z_left)*0.3 + _z7_left;
		}
		else// position control //
		{
			goal_left[0] = ((-(double)acd.controllers[0].pos[2]/10 + 100)*0.6 - trigger_x_left)*0.5 + trigger_arm_x_left;
			goal_left[1] = (-(double)acd.controllers[0].pos[0]/10*0.6 - trigger_y_left)*0.5 + trigger_arm_y_left;
			goal_left[2] = (((double)acd.controllers[0].pos[1]/10+50)*0.6 - trigger_z_left)*0.5 + trigger_arm_z_left;
		}
		*/

		//position control only
		goal_left[0] = ((-(double)acd.controllers[0].pos[2]/10 + 100)*0.6 - trigger_x_left)*0.5 + trigger_arm_x_left;
		goal_left[1] = (-(double)acd.controllers[0].pos[0]/10*0.6 - trigger_y_left)*0.5 + trigger_arm_y_left;
		goal_left[2] = (((double)acd.controllers[0].pos[1]/10+50)*0.6 - trigger_z_left)*0.5 + trigger_arm_z_left;

		
		pred_key_right = curr_key_right;
		curr_key_right = acd.controllers[1].buttons;

		/*
		if (pred_key_right == 0 && curr_key_right == 16)
		{
			vmode_SW_right = true;
			cout<<"right velocity control"<<endl;
		}
		if (pred_key_right == 0 && curr_key_right == 64)
		{
			vmode_SW_right = false;
			cout<<"right position control"<<endl;
		}
		if (vmode_SW_right)// velocity control //
		{
			goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.3 + _x7_right;
			goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.3 + _y7_right;
			goal_right[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*0.3 + _z7_right;
		}
		else// position control //
		{
			goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_right;
			goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_right;
			goal_right[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_right;
		}
		*/

		goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_right;
		goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_right;
		goal_right[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_right;

		//base frame rotation
		//curr_arm_left = curr_rot_left*(!trig_rot_left)*trig_arm_left;
		//curr_arm_right = curr_rot_right*(!trig_rot_right)*trig_arm_right;

		//arm frame rotation reference, equals to equations down
		//curr_arm_left = trig_arm_left*(!trig_rot_left)*curr_rot_left*(!trig_rot_left)*trig_rot_left*(!trig_arm_left)*trig_arm_left;

		//curr_arm_right = trig_arm_right*(!trig_rot_right)*curr_rot_right*(!trig_rot_right)*trig_rot_right*(!trig_arm_right)*trig_arm_right;

		curr_arm_left = trig_arm_left*(!trig_rot_left)*curr_rot_left;

		curr_arm_right = trig_arm_right*(!trig_rot_right)*curr_rot_right;

		//dual_arm control
		if(acd.controllers[0].buttons == 128)
		{
			trigger_down_left = 99;
			trigger_down_right = 99;
			goal_left[0] = _x7_left;
			goal_left[1] = _y7_left-0.5;
			goal_left[2] = _z7_left;

			goal_right[0] = _x7_left;
			goal_right[1] = _y7_right+0.5;
			goal_right[2] = _z7_left;

			curr_arm_left(0,0) = 0;
			curr_arm_left(0,1) = 0;
			curr_arm_left(0,2) = 1;
			curr_arm_left(1,0) = 0;
			curr_arm_left(1,1) = 1;
			curr_arm_left(1,2) = 0;
			curr_arm_left(2,0) = -1;
			curr_arm_left(2,1) = 0;
			curr_arm_left(2,2) = 0;

			curr_arm_right(0,0) = 0;
			curr_arm_right(0,1) = 0;
			curr_arm_right(0,2) = 1;
			curr_arm_right(1,0) = 0;
			curr_arm_right(1,1) = 1;
			curr_arm_right(1,2) = 0;
			curr_arm_right(2,0) = -1;
			curr_arm_right(2,1) = 0;
			curr_arm_right(2,2) = 0;
		}
		
		if (pred_key_right == 0 && curr_key_right == 128)
		{
			//position
			trigger_arm_x_left = _x7_left;
			trigger_arm_y_left = _y7_left;
			trigger_arm_z_left = _z7_left;
			trigger_arm_x_right = _x7_right;
			trigger_arm_y_right = _y7_right;
			trigger_arm_z_right = _z7_right;

			//vector for rotation
			vector(0,0) = (_x7_left - _x7_right)/2;
			vector(1,0) = (_y7_left - _y7_right)/2;
			vector(2,0) = (_z7_left - _z7_right)/2;
			ini_pos_left = vector;
			ini_pos_right = -vector;

			//controller position
			trigger_x_right = (-(double)acd.controllers[1].pos[2]/10 + 100)*0.6;
			trigger_y_right = -(double)acd.controllers[1].pos[0]/10*0.6;
			trigger_z_right = ((double)acd.controllers[1].pos[1]/10 + 50)*0.6;

			//rotation
			hydra_right(0,0) = acd.controllers[1].rot_mat[0][0];
			hydra_right(0,1) = acd.controllers[1].rot_mat[1][0];
			hydra_right(0,2) = acd.controllers[1].rot_mat[2][0];
			hydra_right(1,0) = acd.controllers[1].rot_mat[0][1];
			hydra_right(1,1) = acd.controllers[1].rot_mat[1][1];
			hydra_right(1,2) = acd.controllers[1].rot_mat[2][1];
			hydra_right(2,0) = acd.controllers[1].rot_mat[0][2];
			hydra_right(2,1) = acd.controllers[1].rot_mat[1][2];
			hydra_right(2,2) = acd.controllers[1].rot_mat[2][2];
			trig_rot_right = Rot_Map(hydra_right);
			curr_rot_right = trig_rot_right;

			trig_arm_left = arm_rot_left;
			trig_arm_right = arm_rot_right;
		}
		

		if(acd.controllers[1].buttons == 128)
		{
			trigger_down_left = 99;
			trigger_down_right = 99;
			hydra_right(0,0) = acd.controllers[1].rot_mat[0][0];
			hydra_right(0,1) = acd.controllers[1].rot_mat[1][0];
			hydra_right(0,2) = acd.controllers[1].rot_mat[2][0];
			hydra_right(1,0) = acd.controllers[1].rot_mat[0][1];
			hydra_right(1,1) = acd.controllers[1].rot_mat[1][1];
			hydra_right(1,2) = acd.controllers[1].rot_mat[2][1];
			hydra_right(2,0) = acd.controllers[1].rot_mat[0][2];
			hydra_right(2,1) = acd.controllers[1].rot_mat[1][2];
			hydra_right(2,2) = acd.controllers[1].rot_mat[2][2];
			curr_rot_right = Rot_Map(hydra_right);

			final_pos_left = curr_rot_right*(!trig_rot_right)*ini_pos_left;
			final_pos_right = curr_rot_right*(!trig_rot_right)*ini_pos_right;

			//without rotation
			
			
			//goal_left[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_left;
			//goal_left[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_left - 5;
			//goal_left[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_left;
			//goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_right;
			//goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_right + 5;
			//goal_right[2] = (((double)acd.controllers[1].pos[1]/10 + 50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_right;
			
			goal_left[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_left;
			goal_left[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_left;
			goal_left[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_left;
			goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*0.5 + trigger_arm_x_right;
			goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*0.5 + trigger_arm_y_right;
			goal_right[2] = (((double)acd.controllers[1].pos[1]/10 + 50)*0.6 - trigger_z_right)*0.5 + trigger_arm_z_right;
			
			curr_arm_left(0,0) = 0;
			curr_arm_left(0,1) = 0;
			curr_arm_left(0,2) = 1;
			curr_arm_left(1,0) = 0;
			curr_arm_left(1,1) = 1;
			curr_arm_left(1,2) = 0;
			curr_arm_left(2,0) = -1;
			curr_arm_left(2,1) = 0;
			curr_arm_left(2,2) = 0;

			curr_arm_right(0,0) = 0;
			curr_arm_right(0,1) = 0;
			curr_arm_right(0,2) = 1;
			curr_arm_right(1,0) = 0;
			curr_arm_right(1,1) = 1;
			curr_arm_right(1,2) = 0;
			curr_arm_right(2,0) = -1;
			curr_arm_right(2,1) = 0;
			curr_arm_right(2,2) = 0;
			

			//with rotation
			/*
			goal_left[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*1 + trigger_arm_x_left + final_pos_left(0,0) - ini_pos_left(0,0);
			goal_left[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*1 + trigger_arm_y_left + final_pos_left(1,0) - ini_pos_left(1,0);
			goal_left[2] = (((double)acd.controllers[1].pos[1]/10+50)*0.6 - trigger_z_right)*1 + trigger_arm_z_left + final_pos_left(2,0) - ini_pos_left(2,0);
			goal_right[0] = ((-(double)acd.controllers[1].pos[2]/10 + 100)*0.6 - trigger_x_right)*1 + trigger_arm_x_right + final_pos_right(0,0) - ini_pos_right(0,0);
			goal_right[1] = (-(double)acd.controllers[1].pos[0]/10*0.6 - trigger_y_right)*1 + trigger_arm_y_right + final_pos_right(1,0) - ini_pos_right(1,0);
			goal_right[2] = (((double)acd.controllers[1].pos[1]/10 + 50)*0.6 - trigger_z_right)*1 + trigger_arm_z_right + final_pos_right(2,0) - ini_pos_right(2,0);
			curr_arm_left = trig_arm_left*(!trig_rot_right)*curr_rot_right;
			curr_arm_right = trig_arm_right*(!trig_rot_right)*curr_rot_right;
			*/
		}





		arm_rot_left = inversekinematics(left_arm_ID, goal_left, curr_arm_left, theta_left, _velocity, _wo, _ko, _x7_left, _y7_left, _z7_left, _alpha_left, _beta_left, _gamma_left, _mm, joint_left);

		arm_rot_right = inversekinematics(right_arm_ID, goal_right, curr_arm_right, theta_right, _velocity, _wo, _ko, _x7_right, _y7_right, _z7_right, _alpha_right, _beta_right, _gamma_right, _mm, joint_right);




		//massage
		lPstr0 = to_string((long double)joint_left[0]);
		lPstr1 = to_string((long double)joint_left[1]);
		lPstr2 = to_string((long double)joint_left[2]);
		lPstr3 = to_string((long double)joint_left[3]);
		lPstr4 = to_string((long double)joint_left[4]);
		lPstr5 = to_string((long double)joint_left[5]);
		lPstr6 = to_string((long double)joint_left[6]);
		lPstr7 = to_string((long double)trigger_down_left);
		lPstr8 = to_string((long double)gripper_left);


		rPstr0 = to_string((long double)joint_right[0]);
		rPstr1 = to_string((long double)joint_right[1]);
		rPstr2 = to_string((long double)joint_right[2]);
		rPstr3 = to_string((long double)joint_right[3]);
		rPstr4 = to_string((long double)joint_right[4]);
		rPstr5 = to_string((long double)joint_right[5]);
		rPstr6 = to_string((long double)joint_right[6]);
		rPstr7 = to_string((long double)trigger_down_right);
		rPstr8 = to_string((long double)gripper_right);
		space = ",";
		
		//msg = lPstr0+space+lPstr1+space+lPstr2+space+lPstr3+space+lPstr4+space+lPstr5+space+lPstr6+space+lPstr7+space+lPstr8;
		
		//msg = rPstr0+space+rPstr1+space+rPstr2+space+rPstr3+space+rPstr4+space+rPstr5+space+rPstr6+space+rPstr7+space+rPstr8;
		
		msg = lPstr0+space+lPstr1+space+lPstr2+space+lPstr3+space+lPstr4+space+lPstr5+space+lPstr6+space+lPstr7+space+lPstr8+space+rPstr0+space+rPstr1+space+rPstr2+space+rPstr3+space+rPstr4+space+rPstr5+space+rPstr6+space+rPstr7+space+rPstr8;

		client.Send((char*)msg.c_str());
		//receive joint angles
		client.Recv(recvMsg, DEFAULT_BUFFER_LENGTH);

		//sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_left[0],&theta_left[1],&theta_left[2],&theta_left[3],&theta_left[4],&theta_left[5],&theta_left[6]);

		//sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_right[0],&theta_right[1],&theta_right[2],&theta_right[3],&theta_right[4],&theta_right[5],&theta_right[6]);

		sscanf(recvMsg, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &theta_left[0],&theta_left[1],&theta_left[2],&theta_left[3],&theta_left[4],&theta_left[5],&theta_left[6],&theta_right[0],&theta_right[1],&theta_right[2],&theta_right[3],&theta_right[4],&theta_right[5],&theta_right[6]);

		cout<<goal_right[0]<<setw(12)<<goal_right[1]<<setw(12)<<goal_right[2]<<"\r";

		//Sleep(100);

	}

	return 0;
}


