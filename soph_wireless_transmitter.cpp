#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <openvr.h>
#include <chrono>
#include <thread>
#pragma comment(lib, "Ws2_32.lib")
#include <ws2tcpip.h>
#include <winsock2.h>
#include <unordered_map>
#include <string>
#include "packet.h"

const vr::ETrackedDeviceProperty PROPS_TO_TRACK_I32[] = {
	vr::ETrackedDeviceProperty::Prop_DeviceClass_Int32,
	vr::ETrackedDeviceProperty::Prop_ControllerRoleHint_Int32,
};
const vr::ETrackedDeviceProperty PROPS_TO_TRACK_STRING[] = {
	vr::ETrackedDeviceProperty::Prop_RenderModelName_String,
	vr::ETrackedDeviceProperty::Prop_ModelNumber_String,
	vr::ETrackedDeviceProperty::Prop_TrackingSystemName_String,
	vr::ETrackedDeviceProperty::Prop_IconPathName_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceOff_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceSearching_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceSearchingAlert_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceReady_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceReadyAlert_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceNotReady_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceStandby_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceAlertLow_String,
	vr::ETrackedDeviceProperty::Prop_NamedIconPathDeviceStandbyAlert_String,
	vr::ETrackedDeviceProperty::Prop_RegisteredDeviceType_String,
	vr::ETrackedDeviceProperty::Prop_ResourceRoot_String,
	vr::ETrackedDeviceProperty::Prop_RegisteredDeviceType_String,
	vr::ETrackedDeviceProperty::Prop_InputProfilePath_String,
	vr::ETrackedDeviceProperty::Prop_ControllerType_String,
	vr::ETrackedDeviceProperty::Prop_RenderModelName_String,
	vr::ETrackedDeviceProperty::Prop_ManufacturerName_String,

	vr::ETrackedDeviceProperty::Prop_TrackingFirmwareVersion_String,
	vr::ETrackedDeviceProperty::Prop_HardwareRevision_String,
	vr::ETrackedDeviceProperty::Prop_ConnectedWirelessDongle_String,
};
const vr::ETrackedDeviceProperty PROPS_TO_TRACK_BOOL[] = {
	vr::ETrackedDeviceProperty::Prop_WillDriftInYaw_Bool,
	vr::ETrackedDeviceProperty::Prop_DeviceIsWireless_Bool,
	vr::ETrackedDeviceProperty::Prop_DeviceIsCharging_Bool,
	vr::ETrackedDeviceProperty::Prop_Identifiable_Bool,
};
const vr::ETrackedDeviceProperty PROPS_TO_TRACK_FLOAT[] = {
	vr::ETrackedDeviceProperty::Prop_DeviceBatteryPercentage_Float
};

const vr::ETrackedDeviceProperty PROPS_TO_TRACK_UINT64[] = {
	vr::ETrackedDeviceProperty::Prop_HardwareRevision_Uint64,
	vr::ETrackedDeviceProperty::Prop_FirmwareVersion_Uint64,
	vr::ETrackedDeviceProperty::Prop_FPGAVersion_Uint64,
	vr::ETrackedDeviceProperty::Prop_VRCVersion_Uint64,
	vr::ETrackedDeviceProperty::Prop_RadioVersion_Uint64,
	vr::ETrackedDeviceProperty::Prop_DongleVersion_Uint64,
};

const vr::ETrackedDeviceProperty PROPS_TO_TRACK_M34[] = {
	vr::ETrackedDeviceProperty::Prop_StatusDisplayTransform_Matrix34
};


const size_t PROPS_TO_TRACK_I32_LENGTH = sizeof(PROPS_TO_TRACK_I32) / sizeof(vr::ETrackedDeviceProperty);
const size_t PROPS_TO_TRACK_STRING_LENGTH = sizeof(PROPS_TO_TRACK_STRING) / sizeof(vr::ETrackedDeviceProperty);
const size_t PROPS_TO_TRACK_BOOL_LENGTH = sizeof(PROPS_TO_TRACK_BOOL) / sizeof(vr::ETrackedDeviceProperty);
const size_t PROPS_TO_TRACK_FLOAT_LENGTH = sizeof(PROPS_TO_TRACK_FLOAT) / sizeof(vr::ETrackedDeviceProperty);
const size_t PROPS_TO_TRACK_UINT64_LENGTH = sizeof(PROPS_TO_TRACK_UINT64) / sizeof(vr::ETrackedDeviceProperty);
const size_t PROPS_TO_TRACK_M34_LENGTH = sizeof(PROPS_TO_TRACK_M34) / sizeof(vr::ETrackedDeviceProperty);

struct LastSeenProps {
	int32_t i32s[PROPS_TO_TRACK_I32_LENGTH];
	char strings[PROPS_TO_TRACK_STRING_LENGTH][64];
	bool bools[PROPS_TO_TRACK_BOOL_LENGTH];
	float floats[PROPS_TO_TRACK_FLOAT_LENGTH];
	uint64_t uint64s[PROPS_TO_TRACK_UINT64_LENGTH];
	float value_m34[PROPS_TO_TRACK_UINT64_LENGTH][3][4];
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

bool m34_are_equal(float (*a)[3][4], float (*b)[3][4]){
	for (int32_t i = 0; i < 3; ++i)
	{
		for (int32_t j = 0; j < 4; ++j)
		{
			if (a[i][j] != b[i][j]) return false;
		}
	}

	return true;
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

	std::unordered_map<std::string, LastSeenProps> registered_device_last_seen_props{};

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
				if (device_class == vr::TrackedDeviceClass_GenericTracker || true) { // For now just allow all devices

					char serial[32];
					res = sys->GetStringTrackedDeviceProperty(device_index, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, serial, sizeof(serial));

					if (res == 0) {
						printf("Failed to get serial for tracker at %d\n", device_index);
						continue;
					}

					// If the device was not registered before, register it now
					if (registered_device_last_seen_props.find(serial) == registered_device_last_seen_props.end()) {
						printf(std::format("Initializing tracker's last seen with serial {}\n", serial).c_str());
						registered_device_last_seen_props[serial] = LastSeenProps{};
					}

#pragma region
					LastSeenProps* last_seen_props = &registered_device_last_seen_props[serial];
					// Check each of the props and see if there are any that changed
					for (size_t i = 0; i < PROPS_TO_TRACK_I32_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						int32_t new_value = sys->GetInt32TrackedDeviceProperty(device_index, PROPS_TO_TRACK_I32[i], &e);
						if (e != vr::ETrackedPropertyError::TrackedProp_Success) continue;
						int32_t old_value = last_seen_props->i32s[i];

						if (old_value == new_value) continue;

						last_seen_props->i32s[i] = new_value;

						Packet p{};
						strncpy_s(p.serial, serial, sizeof(serial));
						p.type = PacketType::PROP_UPDATE;
						p.property_update.property = PROPS_TO_TRACK_I32[i];
						p.property_update.type = UpdateValueType::INT32_T;
						p.property_update.value_int32 = new_value;
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}

					for (size_t i = 0; i < PROPS_TO_TRACK_STRING_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						char new_value[64];
						sys->GetStringTrackedDeviceProperty(device_index, PROPS_TO_TRACK_STRING[i], new_value, sizeof(new_value), &e);

						if (e != vr::ETrackedPropertyError::TrackedProp_Success) {
							printf(std::format("Failed to get property {}, reason: '{}'\n", (int)PROPS_TO_TRACK_STRING[i], (int) e).c_str());
							continue;
						}

						char* old_value = last_seen_props->strings[i];

						if (strcmp(old_value, new_value) == 0) {
							printf(std::format("Not setting string property {} to new value '{}'\n", (int)PROPS_TO_TRACK_STRING[i], new_value).c_str());
							continue;
						}
						
						printf(std::format("Updating string property {} to new value '{}'\n", (int)PROPS_TO_TRACK_STRING[i], new_value).c_str());
						strncpy_s(last_seen_props->strings[i], new_value, sizeof(new_value));

						Packet p{};
						strncpy_s(p.serial, serial, sizeof(serial));
						p.type = PacketType::PROP_UPDATE;
						p.property_update.property = PROPS_TO_TRACK_STRING[i];
						p.property_update.type = UpdateValueType::STRING;
						strncpy_s(p.property_update.value_string, new_value, sizeof(new_value));
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}

					for (size_t i = 0; i < PROPS_TO_TRACK_BOOL_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						bool new_value = sys->GetBoolTrackedDeviceProperty(device_index, PROPS_TO_TRACK_BOOL[i], &e);
						if (e != vr::ETrackedPropertyError::TrackedProp_Success) continue;
						bool old_value = last_seen_props->bools[i];

						if (old_value == new_value) continue;

						last_seen_props->bools[i] = new_value;

						Packet p{};
						strncpy_s(p.serial, serial, sizeof(serial));
						p.type = PacketType::PROP_UPDATE;
						p.property_update.property = PROPS_TO_TRACK_BOOL[i];
						p.property_update.type = UpdateValueType::BOOOL;
						p.property_update.value_bool = new_value;
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}

					for (size_t i = 0; i < PROPS_TO_TRACK_FLOAT_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						float new_value = sys->GetFloatTrackedDeviceProperty(device_index, PROPS_TO_TRACK_BOOL[i], &e);
						if (e != vr::ETrackedPropertyError::TrackedProp_Success) continue;
						float old_value = last_seen_props->floats[i];

						if (old_value == new_value) continue;

						last_seen_props->floats[i] = new_value;

						Packet p{};
						strncpy_s(p.serial, serial, sizeof(serial));
						p.type = PacketType::PROP_UPDATE;
						p.property_update.property = PROPS_TO_TRACK_FLOAT[i];
						p.property_update.type = UpdateValueType::FLOOAT;
						p.property_update.value_float = new_value;
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}

					for (size_t i = 0; i < PROPS_TO_TRACK_UINT64_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						uint64_t new_value = sys->GetUint64TrackedDeviceProperty(device_index, PROPS_TO_TRACK_UINT64[i], &e);
						if (e != vr::ETrackedPropertyError::TrackedProp_Success) continue;
						uint64_t old_value = last_seen_props->uint64s[i];

						if (old_value == new_value) continue;

						last_seen_props->uint64s[i] = new_value;

						Packet p{};
						p.type = PacketType::PROP_UPDATE;
						strncpy_s(p.serial, serial, sizeof(serial));
						p.property_update.property = PROPS_TO_TRACK_UINT64[i];
						p.property_update.type = UpdateValueType::UINT64_T;
						p.property_update.value_uint64 = new_value;
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}

					for (size_t i = 0; i < PROPS_TO_TRACK_M34_LENGTH; i++)
					{
						vr::ETrackedPropertyError e;
						vr::HmdMatrix34_t new_value = sys->GetMatrix34TrackedDeviceProperty(device_index, PROPS_TO_TRACK_M34[i], &e);
						if (e != vr::ETrackedPropertyError::TrackedProp_Success) continue;
						float (*old_value)[3][4] = &last_seen_props->value_m34[i];

						if (m34_are_equal(old_value, &new_value.m)) continue;

						memcpy(last_seen_props->value_m34[i], &new_value.m, sizeof(new_value.m));

						Packet p{};
						p.type = PacketType::PROP_UPDATE;
						strncpy_s(p.serial, serial, sizeof(serial));
						p.property_update.property = PROPS_TO_TRACK_UINT64[i];
						p.property_update.type = UpdateValueType::MATRIX34;
						memcpy(p.property_update.value_m34, &new_value.m, sizeof(new_value.m));
						send_and_forget_tcp(sending_tcp_socket, (char*)&p, sizeof(p));
					}
#pragma endregion

					// Finally, send the position updates
					vr::HmdQuaternion_t qRotation = HmdQuaternion_FromMatrix(tracked_poses[device_index].mDeviceToAbsoluteTracking);
					//std::cout << "Quaternion: [w, x, y, z] = [" << qRotation.w << ", " << qRotation.x << ", " << qRotation.y << ", " << qRotation.z << "]\n";
					Packet p{};
					p.type = TRACKER_UPDATE;
					p.tracker_update.deviceIsConnected = tracked_poses[device_index].bDeviceIsConnected;
					p.tracker_update.poseIsValid = tracked_poses[device_index].bPoseIsValid;
					p.tracker_update.qRotation = qRotation;
					p.tracker_update.result = tracked_poses[device_index].eTrackingResult;
					p.tracker_update.vecPosition[0] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[0][3];
					p.tracker_update.vecPosition[1] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[1][3];
					p.tracker_update.vecPosition[2] = tracked_poses[device_index].mDeviceToAbsoluteTracking.m[2][3];
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
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		printf(std::format("Looped (took {})\n", elapsed).c_str());
		//std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		if (elapsed.count() < 50) {
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time) - elapsed);
		}
	}

}
