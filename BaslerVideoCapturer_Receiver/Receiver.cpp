#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <filesystem>
#include <ctime>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <ws2ipdef.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <sys/timeb.h>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/DeviceInfo.h>

#pragma comment(lib, "ws2_32.lib")

#include "PixelFormatAndAoiConfiguration.h"
#include "camNum.h"

#define MAX_CLIENTS 6
#define PORT 4578
#define BUFSIZE 7

using namespace std;
using std::thread;
using namespace Pylon;

size_t num_cameras, num_frames, num_exposure, num_fps;
size_t option_lightSource, option_balanceWhiteAuto, option_deMosaicingMode;
float num_noise, num_sharpness;
string* cam_arr;

void ErrorHandling(const char* message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

size_t camSNMatcher(String_t serialNumber) {
	for (size_t i = 0; i < 20; ++i) {
		if (serialNumber == camNumList[i]) return i + 1;
	}
	return 0;
}

string extractParameters(const string& line) {
	size_t colonPos = line.find(':');
	if (colonPos != string::npos) {
		return line.substr(colonPos + 1);
	}
	return "";
}

int main(int argc, char* argv[]) {
	try {
		// cfg 파일로부터 파라미터 추출
		ifstream cfgFile("./bin/inputParameters.txt");
		string line;
		vector<string> parameters;

		if (cfgFile.is_open()) {
			while (getline(cfgFile, line)) {
				string param = extractParameters(line);
				param.erase(0, param.find_first_not_of(" "));
				param.erase(param.find_last_not_of(" ") + 1);
				parameters.push_back(param);
			}
			cfgFile.close();
		}
		else {
			cerr << "There is no inputParameters.txt file in directory." << endl;
			return 0;
		}

		//파라미터 설정
		num_frames = stoi(parameters[0]);
		string ipAddress = parameters[1];
		num_exposure = stoi(parameters[2]);
		num_fps = stoi(parameters[3]);
		option_lightSource = stoi(parameters[4]);
		option_balanceWhiteAuto = stoi(parameters[5]);
		option_deMosaicingMode = stoi(parameters[6]);
		num_noise = stof(parameters[7]);
		num_sharpness = stof(parameters[8]);

		cout << "All inputed parameters has been set like bellow..." << endl
			<< "Total frames : \t\t\t\t" << num_frames << endl
			<< "IP address of network interface : \t" << ipAddress << endl
			<< "Exposure time : \t\t\t" << num_exposure << " micro_sec" << endl
			<< "Resulting frame rate : \t\t\t" << num_fps << endl
			<< "Light source : \t\t\t\t" << (option_lightSource == 0 ? "off" : option_lightSource == 1 ? "Daylight 5000K" : option_lightSource == 2 ? "Daylight 6400K" : option_lightSource == 3 ? "Tungsten 2800K" : "") << endl
			<< "Balance white auto : \t\t\t" << (option_balanceWhiteAuto == 0 ? "off" : option_balanceWhiteAuto == 1 ? "once" : option_balanceWhiteAuto == 2 ? "continuous" : "") << endl
			<< "De-mosaicing mode : \t\t\t" << (option_deMosaicingMode == 0 ? "off" : option_deMosaicingMode == 1 ? "on" : "") << endl
			<< "Noise reduction value : \t\t" << num_noise << endl
			<< "Sharpness enhancement value : \t\t" << num_sharpness << endl
			<< endl;

		PylonInitialize();

		// UDP 멀티캐스트
		int retval;
		cout << "Initializing windows socket..." << endl;
		WSADATA wsa;
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) ErrorHandling("WSAStartup() failed.");

		// UDP 소켓 설정
		cout << "Generating UDP socket..." << endl;
		SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (sock == INVALID_SOCKET) ErrorHandling("socket() failed.");

		// SO_REUSEADDR
		cout << "Setting sockop - SO_REUSEADDR..." << endl;
		bool optval = TRUE;
		retval = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&optval, sizeof(optval));
		if (retval == SOCKET_ERROR) ErrorHandling("setsockopt()-SO_REUSEADDR");

		// 구조체 할당
		cout << "Assigning structure..." << endl;
		SOCKADDR_IN serveraddr;
		ZeroMemory(&serveraddr, sizeof(serveraddr));
		serveraddr.sin_family = AF_INET;
		serveraddr.sin_port = htons(PORT);
		serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

		// bind()
		cout << "Connecting with bind()..." << endl;
		retval = bind(sock, (SOCKADDR*)&serveraddr, sizeof(serveraddr));
		if (retval == SOCKET_ERROR) ErrorHandling("connect() failed.");

		//  멀티캐스트 그룹 가입
		cout << "Joining multicast group..." << endl;
		int enableMulticast = 1;
		struct ip_mreq mreq;
		mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.123");
		mreq.imr_interface.s_addr = inet_addr(ipAddress.c_str());
		//mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		retval = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
		if (retval == SOCKET_ERROR) ErrorHandling("setsockop()-ip_mreq failed.");

		// 데이터 통신에 사용할 변수
		SOCKADDR_IN peeraddr;
		int addrlen = sizeof(peeraddr);
		char buf[BUFSIZE + 1];

		// 영상 취득 설정
		const int ImageBufferSize = 1952 * 1088 * 3;
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		DeviceInfoList_t devices;
		if (tlFactory.EnumerateDevices(devices) == 0) {
			throw RUNTIME_EXCEPTION("No camera present.");
		}

		CBaslerUniversalInstantCameraArray cameras(devices.size());
		size_t num_cameras = cameras.GetSize();
		auto ImageBuffer = new const char* [num_frames * num_cameras];
		vector<ofstream> files(MAX_CLIENTS + 1);
		time_t filetime = time(NULL);
		struct tm* ft = localtime(&filetime);
		string dirName("./bin/");
		dirName.append(to_string(ft->tm_year + 1900)).append(to_string(ft->tm_mon + 1)).append(to_string(ft->tm_mday)).append("/");//.append(machineName).append("/");
		cout << "Saving directory : " << dirName << endl << endl;
		filesystem::create_directory(dirName.c_str());

		auto TimeStamp = new uint64_t[num_frames * num_cameras];
		ofstream TimeStampLog;
		string logName = dirName;
		logName.append("TimeStampLog.txt");
		TimeStampLog.open(logName);
		TimeStampLog << "Time stamp in acA2500-60uc: 1GHz(=1,000,000,000 ticks per second, 1 tick = 1 ns)" << endl << endl;

		size_t cam_num;
		for (size_t i = 0; i < num_cameras; ++i) {
			cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
			cameras[i].RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
			cameras[i].RegisterConfiguration(new CPixelFormatAndAoiConfiguration, RegistrationMode_Append, Cleanup_Delete);

			cam_num = camSNMatcher(cameras[i].GetDeviceInfo().GetSerialNumber());

			string fileName = dirName;
			fileName.append("rgb_camera").append(to_string(cam_num)).append("_1952x1088_").append(to_string(ft->tm_year + 1900)).append(to_string(ft->tm_mon + 1)).append(to_string(ft->tm_mday)).append(".rgb");
			cout << "file opened : " << fileName << endl;
			files[i] = ofstream(fileName.c_str(), ios::out | ios::binary);
		}

		CGrabResultPtr ptrGrabResult;
		bool* AccessFlag = new bool[num_frames * num_cameras] {false};
		size_t totalFrames = num_frames * num_cameras, bufferIndex = 0;

		cameras.Open();
		//cameras.StartGrabbing();
		cameras.StartGrabbing(GrabStrategy_LatestImageOnly);
		//cameras.StartGrabbing(GrabStrategy_OneByOne);
		//cameras.StartGrabbing(GrabStrategy_LatestImages);

		// 영상 취득 시작
		struct timeb now;
		struct tm* nowtime;
		cout << endl << "Waiting for Sender to send EXECUTE message..." << endl;
		while (1) {
			retval = recvfrom(sock, buf, BUFSIZE, 0, (SOCKADDR*)&peeraddr, &addrlen);
			if (strcmp(buf, "execute") == 0) break;
		}
		ftime(&now);
		nowtime = localtime(&now.time);
		TimeStampLog << "Started: " << nowtime->tm_hour << ":" << nowtime->tm_min << ":" << nowtime->tm_sec << ":" << now.millitm << endl << endl;

		thread thread_ImageToBuffer([&]() {
			cout << "Thread 1 : Image Grab and Put to Buffer" << endl << endl;
			for (size_t frameIndex = 0; frameIndex < totalFrames; frameIndex += num_cameras) {
				for (size_t i = 0; i < num_cameras; ++i) {
					cameras[i].RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
					ImageBuffer[frameIndex + i] = (const char*)ptrGrabResult->GetBuffer();
					AccessFlag[frameIndex + i] = true;
					TimeStamp[frameIndex + i] = ptrGrabResult->GetTimeStamp();
				}
			}
			cout << "Thread 1 finished" << endl << endl;
			});

		thread thread_BufferToFile([&]() {
			cout << "Thread 2 : Buffer to File" << endl << endl;
			size_t bufferIndex = 0, totalFrames = num_frames * num_cameras;
			while (bufferIndex < totalFrames) {
				if (AccessFlag[bufferIndex] == true) {
					files[bufferIndex % num_cameras].write(ImageBuffer[bufferIndex], ImageBufferSize);
					++bufferIndex;
				}
			}
			cout << "Thread 2 finished" << endl << endl;
			});

		// Main thread
		cout << "Main : Software Trigger Ready and Execute" << endl << endl;
		for (size_t frameIndex = 0; frameIndex < num_frames; ++frameIndex) {
			retval = recvfrom(sock, buf, BUFSIZE, 0, (SOCKADDR*)&peeraddr, &addrlen);
			if (strcmp(buf, "execute") == 0) {
				for (size_t i = 0; i < num_cameras; ++i) {
					if (cameras[i].WaitForFrameTriggerReady(1000, TimeoutHandling_ThrowException)) {
						cameras[i].ExecuteSoftwareTrigger();
					}
				}
			}
		}

		thread_ImageToBuffer.join();
		thread_BufferToFile.join();

		cout << "All threads have been finished." << endl << endl;

		cout << "Recording sync diff to log file..." << endl << endl;

		TimeStampLog << endl;
		float* avg_diff = new float[num_frames]();
		float** cam_diff = new float* [num_frames];
		for (size_t i = 0; i < num_frames; ++i) cam_diff[i] = new float[num_cameras];
		for (size_t frameIndex = 0; frameIndex < totalFrames; frameIndex += num_cameras) {
			TimeStampLog << frameIndex / num_cameras << "th frame: " << endl;
			for (size_t i = 0; i < num_cameras; ++i) {
				TimeStampLog << "cam" << i << " : " << TimeStamp[frameIndex + i] << " ns";
				if (frameIndex > 0) {
					cam_diff[frameIndex / num_cameras][i] = (float)(TimeStamp[frameIndex + i] - TimeStamp[frameIndex + i - num_cameras]) / 1000000;
					TimeStampLog << "\t (diff: " << cam_diff[frameIndex / num_cameras][i] << " ms)" << endl;
					avg_diff[frameIndex / num_cameras] += cam_diff[frameIndex / num_cameras][i];
				}
				else TimeStampLog << endl;
			}
			TimeStampLog << endl;
		}

		TimeStampLog << endl << "Average time diff and time error of cameras" << endl << endl;

		TimeStampLog << "frameIndex\ttime diff\t";
		for (size_t i = 1; i < num_cameras; ++i) TimeStampLog << "cam" << i - 1 << "," << i << " error\t";

		float total_avg_diff = 0, total_avg_error = 0;
		TimeStampLog << endl;
		for (size_t frameIndex = 1; frameIndex < num_frames; ++frameIndex) {
			TimeStampLog << frameIndex << "th frame:\t " << avg_diff[frameIndex] / num_cameras << " ms";
			total_avg_diff += avg_diff[frameIndex] / num_cameras;
			for (size_t i = 1; i < num_cameras; ++i) {
				TimeStampLog << "\t" << abs(cam_diff[frameIndex][i] - cam_diff[frameIndex][i - 1]) << " ms";
				total_avg_error += abs(cam_diff[frameIndex][i] - cam_diff[frameIndex][i - 1]);
			}
			TimeStampLog << endl;
		}
		TimeStampLog << endl << "Total average time diff:\t " << total_avg_diff / (num_frames - 1) << " ms" << endl;
		TimeStampLog << endl << "Total average time error:\t " << total_avg_error / ((num_frames - 1) * (num_cameras - 1)) << " ms" << endl;

		cout << "Recording sync diff has been finished." << endl << endl;

		for (size_t i = 0; i < num_cameras; ++i) {
			cameras[i].StopGrabbing();
			cameras[i].Close();
		}
		for (size_t i = 0; i < MAX_CLIENTS; ++i) {
			files[i].close();
		}

		for (size_t i = 0; i < num_frames; ++i) {
			delete[] ImageBuffer[i];
			delete[] avg_diff;
			delete[] cam_diff[i];
		}

		delete[] ImageBuffer;
		delete[] AccessFlag;
		delete[] TimeStamp;
		delete[] cam_diff;

		TimeStampLog.close();

		PylonTerminate();

		// closesocket()
		cout << "Closing socket..." << endl;
		closesocket(sock);

		// Winsock 종료
		cout << "Terminating windows socket..." << endl;
		WSACleanup();

	}
	catch (GenericException& e) {
		cerr << "An exception occurred." << endl << e.GetDescription() << endl;
		return 1;
	}
}