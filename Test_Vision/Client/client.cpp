#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
    //Robots::SendRequest(argc, argv, "/usr/Robots/resource/HexapodIII/HexapodIII.xml");
    Robots::SendRequest(argc, argv, "../resource/client.xml");

	return 0;
}
