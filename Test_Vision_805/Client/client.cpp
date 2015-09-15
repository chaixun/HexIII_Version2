#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
    //Robots::SendRequest(argc, argv, "/usr/Robots/resource/HexapodIII/HexapodIII.xml");
    Robots::SendRequest(argc, argv, "/home/hex/git_cx/Test_Vision_805/resource/HexapodIII.xml");

	return 0;
}
