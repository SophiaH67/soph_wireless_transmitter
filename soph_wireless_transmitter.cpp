#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <openvr.h>
#include <chrono>
#include <thread>
#pragma comment(lib, "Ws2_32.lib")
#include <ws2tcpip.h>
#include <winsock2.h>

enum PacketType {
	TRACKER_UPDATE
};

struct Packet {
	PacketType packet_type;
	char packet[512];
};

// Required params for the driver to register and track a tracker
struct TrackerUpdatePacket {
	double vecPosition[3];
	double vecVelocity[3];
	double vecAcceleration[3];
	double vecAngularVelocity[3];
	double vecAngularAcceleration[3];
	vr::HmdQuaternion_t qRotation;
	vr::ETrackingResult result;
	bool poseIsValid;
	bool deviceIsConnected;
	char serial[32]; // As identification of device
};


// Stolen from valve docs
template < class T >
vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

sockaddr_in get_sending_sockaddr(const char ip[32]) {
	struct sockaddr_in target_addr {};
	inet_pton(AF_INET, ip, &(target_addr.sin_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_port = htons(6767);

	return target_addr;
}

SOCKET get_sending_socket() {

	SOCKET sending_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	return sending_socket;
}

int main(int argc, char* argv[])
{

	if (argc != 3) {
		printf("Usage: %s <ip address> <sleep time ms>", argv[0]);
		return 1;
	}

	int sleep_time = atoi(argv[2]);
	if (sleep_time == 0) {
		printf("Refresh rate '%s' is not valid", argv[2]);
		return 1;
	}

	char* ip_addr = argv[1];


	vr::EVRInitError error;
	vr::IVRSystem* sys = vr::VR_Init(&error, vr::VRApplication_Overlay);
	if (error != 0) {
		printf("Error initializng: %s\n", VR_GetVRInitErrorAsSymbol(error));
	}

	vr::VROverlayHandle_t handle;
	vr::VROverlay()->CreateOverlay("soph_wireless_transmitter", "Soph Wireless Transmitter", &handle);

	std::cout << "Hello World!\n";

	// Init socket garbage
	WSADATA wsa_data;
	int res = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (res != 0) {
		wprintf(L"WSAStartup failed with error: %d\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}
	SOCKET sending_socket = get_sending_socket();
	sockaddr_in target_addr = get_sending_sockaddr(ip_addr);

	while (true) {
		vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount] = {};
		int tracked_poses_length = sizeof(tracked_poses) / sizeof(tracked_poses[0]);
		//vr::EVRCompositorError e = vr::VRCompositor()->WaitGetPoses(tracked_poses, tracked_poses_length, nullptr, 0);
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.f, tracked_poses, tracked_poses_length);
		/*if (e != vr::EVRCompositorError::VRCompositorError_None) {
			printf("Got error %d while trying to get poses\n", e);
			std::this_thread::sleep_for(std::chrono::seconds(10));
			continue;
		}*/

		// Skip 1 due to HMD being 0 always
		for (int i = 1; i < tracked_poses_length; i++)
		{
			// If it's not currently connected we don't care
			if (vr::VRSystem()->IsTrackedDeviceConnected(i)) {
				// Only trackers
				uint32_t device_class = vr::VRSystem()->GetInt32TrackedDeviceProperty(i, vr::Prop_DeviceClass_Int32);
				if (device_class == vr::TrackedDeviceClass_GenericTracker || true) { // For now just allow all devices

					char serial[32];
					res = sys->GetStringTrackedDeviceProperty(i, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, serial, sizeof(serial));

					if (res == 0) {
						printf("Failed to get serial for tracker at %d", i);
						continue;
					}


					// Print the serial number and it's matrix
					/*printf("Matrix for device with serial '%s':\n", serial);
					for (int j = 0; j < 3; j++) {
						for (int k = 0; k < 4; k++) {
							std::cout << tracked_poses[i].mDeviceToAbsoluteTracking.m[j][k] << "\t";
						}
						std::cout << std::endl;
					}*/
					// Convert it with magic function
					vr::HmdQuaternion_t qRotation = HmdQuaternion_FromMatrix(tracked_poses[i].mDeviceToAbsoluteTracking);
					//std::cout << "Quaternion: [w, x, y, z] = [" << qRotation.w << ", " << qRotation.x << ", " << qRotation.y << ", " << qRotation.z << "]\n";
					TrackerUpdatePacket update_packet{};
					update_packet.deviceIsConnected = tracked_poses[i].bDeviceIsConnected;
					update_packet.poseIsValid = tracked_poses[i].bPoseIsValid;
					update_packet.qRotation = qRotation;
					update_packet.result = tracked_poses[i].eTrackingResult;
					update_packet.vecPosition[0] = tracked_poses[i].mDeviceToAbsoluteTracking.m[0][3];
					update_packet.vecPosition[1] = tracked_poses[i].mDeviceToAbsoluteTracking.m[1][3];
					update_packet.vecPosition[2] = tracked_poses[i].mDeviceToAbsoluteTracking.m[2][3];
					memcpy(update_packet.serial, serial, sizeof(serial));
					//std::cout << "Pos [x,y,z] = [" << tracked_poses[i].mDeviceToAbsoluteTracking.m[0][3] << ", " << tracked_poses[i].mDeviceToAbsoluteTracking.m[1][3] << ", " << tracked_poses[i].mDeviceToAbsoluteTracking.m[2][3] << "]\n";

					Packet packet{};
					packet.packet_type = TRACKER_UPDATE;
					memcpy(packet.packet, &update_packet, sizeof(update_packet));
					res = sendto(sending_socket, (char*)&packet, sizeof(packet), 0, (SOCKADDR*)&target_addr, sizeof(target_addr));
					if (res == SOCKET_ERROR) {
						wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
					}
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		printf("Looped");
	}

}
