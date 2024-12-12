#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <openvr.h>
#include <chrono>
#include <thread>
#pragma comment(lib, "Ws2_32.lib")
#include <ws2tcpip.h>
#include <winsock2.h>
#include <optional>
#include <set>
#include <algorithm>
#include <string>
#include "packet.h"

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

SOCKET get_udp_sending_socket() {
	SOCKET sending_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	return sending_socket;
}

SOCKET create_and_connect_tcp_sending_socket(sockaddr_in server_addr) {
	SOCKET sending_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (connect(sending_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
		printf("Connection failed: %d\n", WSAGetLastError());
		closesocket(sending_socket);
		WSACleanup();
		exit(1);
	}

	return sending_socket;
}

void send_and_forget_tcp(SOCKET sending_socket, const char* message, int length) {
	if (send(sending_socket, message, length, 0) == SOCKET_ERROR) {
		printf("Send failed: %d\n", WSAGetLastError());
		closesocket(sending_socket);
		WSACleanup();
		exit(1);
	}
	else {
		printf("Sent TCP packet!!\n");
	}
}

bool has_prop(const vr::ETrackedDeviceProperty props[], size_t size, vr::ETrackedDeviceProperty target) {
	for (size_t i = 0; i < size; ++i) {
		if (props[i] == target) {
			return true;
		}
	}
	return false;
}

PropertyUpdatePacket generate_update_packet_from_property(int device_index, vr::ETrackedDeviceProperty prop) {
	PropertyUpdatePacket p{};
	p.property = prop;

	vr::ETrackedPropertyError e;

	if (has_prop(PROPS_TO_TRACK_I32, PROPS_TO_TRACK_I32_LENGTH, prop)) {
		p.value_int32 = vr::VRSystem()->GetInt32TrackedDeviceProperty(device_index, prop, &e);
		p.type = PropValueType::INT32_T;
	}

	if (has_prop(PROPS_TO_TRACK_STRING, PROPS_TO_TRACK_STRING_LENGTH, prop)) {
		vr::VRSystem()->GetStringTrackedDeviceProperty(device_index, prop, p.value_string, sizeof(p.value_string), &e);
		p.type = PropValueType::STRING;
	}

	if (has_prop(PROPS_TO_TRACK_BOOL, PROPS_TO_TRACK_BOOL_LENGTH, prop)) {
		p.value_bool= vr::VRSystem()->GetBoolTrackedDeviceProperty(device_index, prop, &e);
		p.type = PropValueType::BOOOL;
	}

	if (has_prop(PROPS_TO_TRACK_FLOAT, PROPS_TO_TRACK_FLOAT_LENGTH, prop)) {
		p.value_float = vr::VRSystem()->GetFloatTrackedDeviceProperty(device_index, prop, &e);
		p.type = PropValueType::FLOOAT;
	}

	if (has_prop(PROPS_TO_TRACK_UINT64, PROPS_TO_TRACK_UINT64_LENGTH, prop)) {
		p.value_uint64 = vr::VRSystem()->GetUint64TrackedDeviceProperty(device_index, prop, &e);
		p.type = PropValueType::UINT64_T;
	}

	if (has_prop(PROPS_TO_TRACK_M34, PROPS_TO_TRACK_M34_LENGTH, prop)) {
		vr::HmdMatrix34_t m = vr::VRSystem()->GetMatrix34TrackedDeviceProperty(device_index, prop, &e);
		memcpy(p.value_m34, m.m, sizeof(vr::HmdMatrix34_t));
		p.type = PropValueType::MATRIX34;
	}


	if (e != vr::ETrackedPropertyError::TrackedProp_Success) {
		p.type = PropValueType::UNINITIALIZED;
	};

	return p;
}

int main(int argc, char* argv[])
{

	if (argc != 3) {
		printf("Usage: %s <ip address> <sleep time ms>\n", argv[0]);
		return 1;
	}

	int sleep_time = atoi(argv[2]);
	if (sleep_time == 0) {
		printf("Refresh rate '%s' is not valid\n", argv[2]);
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
	SOCKET sending_udp_socket = get_udp_sending_socket();
	sockaddr_in target_addr = get_sending_sockaddr(ip_addr);
	SOCKET sending_tcp_socket = create_and_connect_tcp_sending_socket(target_addr);

	std::set<std::string> registered_devices{};

	while (true) {
		auto start = std::chrono::high_resolution_clock::now();

		vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount] = {};
		int tracked_poses_length = sizeof(tracked_poses) / sizeof(tracked_poses[0]);

		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.f, tracked_poses, tracked_poses_length);

		// Skip 1 due to HMD being 0 always
		for (int device_index = 1; device_index < tracked_poses_length; device_index++)
		{
			// If it's not currently connected we don't care
			if (vr::VRSystem()->IsTrackedDeviceConnected(device_index)) {
				// Only trackers
				uint32_t device_class = vr::VRSystem()->GetInt32TrackedDeviceProperty(device_index, vr::Prop_DeviceClass_Int32);
				if (device_class == vr::TrackedDeviceClass_GenericTracker || device_class == vr::TrackedDeviceClass::TrackedDeviceClass_Controller) { // For now just allow all devices

					char serial[32];
					res = sys->GetStringTrackedDeviceProperty(device_index, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, serial, sizeof(serial));

					if (res == 0) {
						printf("Failed to get serial for tracker at %d\n", device_index);
						continue;
					}

#pragma region Registration stuff
					// If the device was not registered before, register it now
					if (!registered_devices.contains(serial)) {
						printf(std::format("Registering tracker with serial {}\n", serial).c_str());
						DeviceRegisterPacket register_packet{};
						registered_devices.insert(serial);


						// Check each of the props and see if there are any that changed
						for (size_t i = 0; i < PROPS_TO_TRACK_I32_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_I32[i]);
							memcpy(&register_packet.properties[i], &p, sizeof(p));
						}

						for (size_t i = 0; i < PROPS_TO_TRACK_STRING_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_STRING[i]);
							memcpy(&register_packet.properties[i + PROPS_TO_TRACK_I32_LENGTH], &p, sizeof(p));
						}

						for (size_t i = 0; i < PROPS_TO_TRACK_BOOL_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_BOOL[i]);
							memcpy(&register_packet.properties[i + PROPS_TO_TRACK_I32_LENGTH + PROPS_TO_TRACK_STRING_LENGTH], &p, sizeof(p));
						}

						for (size_t i = 0; i < PROPS_TO_TRACK_FLOAT_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_FLOAT[i]);
							memcpy(&register_packet.properties[i + PROPS_TO_TRACK_I32_LENGTH + PROPS_TO_TRACK_STRING_LENGTH + PROPS_TO_TRACK_BOOL_LENGTH], &p, sizeof(p));
						}

						for (size_t i = 0; i < PROPS_TO_TRACK_UINT64_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_UINT64[i]);
							memcpy(&register_packet.properties[i + PROPS_TO_TRACK_I32_LENGTH + PROPS_TO_TRACK_STRING_LENGTH + PROPS_TO_TRACK_BOOL_LENGTH + PROPS_TO_TRACK_FLOAT_LENGTH], &p, sizeof(p));
						}

						for (size_t i = 0; i < PROPS_TO_TRACK_M34_LENGTH; i++)
						{
							PropertyUpdatePacket p = generate_update_packet_from_property(device_index, PROPS_TO_TRACK_M34[i]);
							memcpy(&register_packet.properties[i + PROPS_TO_TRACK_I32_LENGTH + PROPS_TO_TRACK_STRING_LENGTH + PROPS_TO_TRACK_BOOL_LENGTH + PROPS_TO_TRACK_FLOAT_LENGTH + PROPS_TO_TRACK_UINT64_LENGTH], &p, sizeof(p));
						}

						BigPacket p{};
						p.type = PacketType::BIG_DEVICE_REGISTER;
						strncpy_s(p.serial, serial, sizeof(serial));
						register_packet.device_class = (vr::ETrackedDeviceClass)sys->GetInt32TrackedDeviceProperty(device_index, vr::ETrackedDeviceProperty::Prop_DeviceClass_Int32);
						std::memcpy(&p.device_register, &register_packet, sizeof(DeviceRegisterPacket));

						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
						printf(std::format("Register packet sent for serial '{}' (type: {}). Dc: {}\n", p.serial, (int)p.type, (int)p.device_register.device_class).c_str());
						continue;
					}
#pragma endregion

#pragma region Prop update stuff
					vr::VREvent_t e;
					while (sys->PollNextEvent(&e, sizeof(e))) {
						if (e.eventType != vr::EVREventType::VREvent_TrackedDeviceUpdated) continue;
						vr::VREvent_Property_t* prop = &e.data.property;
						printf(std::format("Got change for event {}\n", e.eventType).c_str());
					}
#pragma endregion
					// Finally, send the position updates
					vr::HmdQuaternion_t qRotation = HmdQuaternion_FromMatrix(tracked_poses[device_index].mDeviceToAbsoluteTracking);
					//std::cout << "Quaternion: [w, x, y, z] = [" << qRotation.w << ", " << qRotation.x << ", " << qRotation.y << ", " << qRotation.z << "]\n";
					Packet p{};
					p.type = SMALL_TRACKER_UPDATE;
					p.tracker_update.deviceIsConnected = tracked_poses[device_index].bDeviceIsConnected;
					p.tracker_update.poseIsValid = tracked_poses[device_index].bPoseIsValid;
					p.tracker_update.qRotation = qRotation;
					p.tracker_update.result = tracked_poses[device_index].eTrackingResult;
					p.tracker_update.vecPosition[0] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[0][3];
					p.tracker_update.vecPosition[1] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[1][3];
					p.tracker_update.vecPosition[2] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[2][3];

					auto time = std::chrono::system_clock::now();
					auto unix = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch());
					p.tracker_update.unixTimestamp = unix.count();

					strncpy_s(p.serial, serial, sizeof(serial));
					//std::cout << "Pos [x,y,z] = [" << tracked_poses[i].mDeviceToAbsoluteTracking.m[0][3] << ", " << tracked_poses[i].mDeviceToAbsoluteTracking.m[1][3] << ", " << tracked_poses[i].mDeviceToAbsoluteTracking.m[2][3] << "]\n";

					res = sendto(sending_udp_socket, (char*)&p, sizeof(p), 0, (SOCKADDR*)&target_addr, sizeof(target_addr));
					if (res == SOCKET_ERROR) {
						wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
					}
				}
			}
		}
		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed_before = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		//std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		if (elapsed_before.count() < sleep_time) {
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time) - elapsed_before);
		}
		auto end_after = std::chrono::high_resolution_clock::now();
		auto elapsed_after = std::chrono::duration_cast<std::chrono::milliseconds>(end_after - start);
		printf(std::format("Looped (took {})\n", elapsed_after).c_str());

	}
}
