//https://support.thinklucid.com/app-note-helios-3d-point-cloud-with-rgb-color/#gs-opencv2
//https://support.thinklucid.com/using-opencv-with-arena-sdk-on-linux/
//#include "stdafx.h"
#include "ArenaApi.h"

#include "signal.h"
#include "unistd.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#define TAB1 "  "			
#define TAB2 "    "
#define TAB3 "      "

// pixel format
#define PIXEL_FORMAT "Coord3D_ABCY16"
// image timeout
#define IMAGE_TIMEOUT 2000

struct PointData
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t intensity;
};

void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

void stream_data(Arena::IDevice* pDevice){
    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();
    
	//GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");
    //std::cout<<"Operation mode "<<operatingModeInitial<<std::endl;

	// change operating mode from Far Mode to Near Mode
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1500mm");

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	// get image
	pDevice->StartStream();
	while (true){
		Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
		std::cout<<pImage<<std::endl;
        //sleep(1.0);
	}
	


    //GenICam::gcstring operatingModeChanged = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");
    //std::cout<<"Operation mode "<<operatingModeChanged<<std::endl;
}

int show_image(){
	cv::Mat image = cv::imread("lena.png");

    // Check for failure
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        std::cin.get(); //wait for any key press
        return -1;
    }

    std::string windowName = "window"; //Name of the window
    cv::namedWindow(windowName); // Create a window
    cv::imshow(windowName, image); // Show our image inside the created window.
    cv::waitKey(0); // Wait for any keystroke in the window
    cv::destroyWindow(windowName); //destroy the created window
    return 1;
}

int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;
	std::cout << "ArenaAPI Hello World\n";
	signal(SIGINT, signal_callback_handler);
	
	try
	{
		// run example
		std::cout << "Commence example\n\n";
        // prepare example
		Arena::ISystem* pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(100);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}
		Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);
        stream_data(pDevice);

		pSystem->DestroyDevice(pDevice);
		Arena::CloseSystem(pSystem);
        
		std::cout << "\nExample complete\n";
	}
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "Standard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "Unexpected exception thrown\n";
		exceptionThrown = true;
	}

	std::cout << "Press enter to complete\n";
	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}