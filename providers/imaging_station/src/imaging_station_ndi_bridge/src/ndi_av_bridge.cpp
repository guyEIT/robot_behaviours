#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "Processing.NDI.Lib.h"

namespace {

using Clock = std::chrono::steady_clock;
using NDIlib_v6_load_ = const NDIlib_v6 *(*)(void);

constexpr uint32_t kMagic = 0x31425641;  // "AVB1"
constexpr uint16_t kVersion = 1;

enum PacketType : uint16_t {
	kStatus = 1,
	kVideo = 2,
	kAudio = 3,
	kEos = 4,
	kError = 5,
};

struct __attribute__((packed)) PacketHeader {
	uint32_t magic;
	uint16_t version;
	uint16_t packet_type;
	uint64_t payload_size;
	int64_t timestamp;
	uint32_t width;
	uint32_t height;
	uint32_t stride;
	uint32_t fourcc;
	uint32_t fps_n;
	uint32_t fps_d;
	uint32_t sample_rate;
	uint32_t channels;
	uint32_t samples;
	uint32_t flags;
};

static_assert(sizeof(PacketHeader) == 64, "PacketHeader must stay packed");

std::atomic<bool> g_running{true};

struct Args {
	std::string ndi_name;
	std::string url_address;
	std::string socket_path;
	std::string recv_name = "Experiment Collector SDK AV Bridge";
	double startup_timeout_seconds = 12.0;
	int max_fps = 0;           // 0 = no limit (send every frame from NDI)
	bool use_uyvy = true;      // true = UYVY (2 bytes/pixel), false = BGRX (4 bytes/pixel)
};

void HandleSignal(int)
{
	g_running.store(false);
}

void Usage(const char *argv0)
{
	std::cerr << "Usage: " << argv0 << " --ndi-name <name> --socket-path <path> [options]\n"
		  << "Options:\n"
		  << "  --url-address <host:port>\n"
		  << "  --recv-name <name>\n"
		  << "  --startup-timeout <seconds>\n"
		  << "  --max-fps <N>           Limit video frame rate (0=unlimited, default)\n"
		  << "  --color-format <fmt>    uyvy (2 bpp, default) or bgrx (4 bpp)\n";
}

bool ParseArgs(int argc, char **argv, Args *args)
{
	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];
		auto need_value = [&](const char *flag) -> const char * {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for " << flag << "\n";
				return nullptr;
			}
			return argv[++i];
		};

		if (arg == "--ndi-name") {
			auto value = need_value("--ndi-name");
			if (!value)
				return false;
			args->ndi_name = value;
		} else if (arg == "--url-address") {
			auto value = need_value("--url-address");
			if (!value)
				return false;
			args->url_address = value;
		} else if (arg == "--socket-path") {
			auto value = need_value("--socket-path");
			if (!value)
				return false;
			args->socket_path = value;
		} else if (arg == "--recv-name") {
			auto value = need_value("--recv-name");
			if (!value)
				return false;
			args->recv_name = value;
		} else if (arg == "--startup-timeout") {
			auto value = need_value("--startup-timeout");
			if (!value)
				return false;
			args->startup_timeout_seconds = std::strtod(value, nullptr);
		} else if (arg == "--max-fps") {
			auto value = need_value("--max-fps");
			if (!value)
				return false;
			args->max_fps = std::atoi(value);
		} else if (arg == "--color-format") {
			auto value = need_value("--color-format");
			if (!value)
				return false;
			std::string fmt = value;
			if (fmt == "uyvy")
				args->use_uyvy = true;
			else if (fmt == "bgrx")
				args->use_uyvy = false;
			else {
				std::cerr << "Unknown color format: " << fmt << " (use uyvy or bgrx)\n";
				return false;
			}
		} else if (arg == "--help" || arg == "-h") {
			Usage(argv[0]);
			std::exit(0);
		} else {
			std::cerr << "Unknown argument: " << arg << "\n";
			return false;
		}
	}

	if (args->ndi_name.empty()) {
		std::cerr << "--ndi-name is required\n";
		return false;
	}
	if (args->socket_path.empty()) {
		std::cerr << "--socket-path is required\n";
		return false;
	}
	return true;
}

void *LoadLibndiHandle()
{
	const char *runtime_dir = std::getenv("NDI_RUNTIME_DIR_V6");
	if (runtime_dir && runtime_dir[0] != '\0') {
		std::string candidate = std::string(runtime_dir) + "/libndi.so.6";
		if (void *handle = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL)) {
			return handle;
		}
	}
	return dlopen("libndi.so.6", RTLD_NOW | RTLD_LOCAL);
}

bool WriteAll(int fd, const uint8_t *data, size_t size)
{
	size_t offset = 0;
	while (offset < size) {
		const auto written = ::write(fd, data + offset, size - offset);
		if (written < 0) {
			return false;
		}
		offset += static_cast<size_t>(written);
	}
	return true;
}

bool SendPacket(int fd, PacketType packet_type, const uint8_t *payload, size_t payload_size, int64_t timestamp,
		uint32_t width = 0, uint32_t height = 0, uint32_t stride = 0, uint32_t fourcc = 0,
		uint32_t fps_n = 0, uint32_t fps_d = 0, uint32_t sample_rate = 0, uint32_t channels = 0,
		uint32_t samples = 0, uint32_t flags = 0)
{
	const PacketHeader header = {
		kMagic,
		kVersion,
		packet_type,
		payload_size,
		timestamp,
		width,
		height,
		stride,
		fourcc,
		fps_n,
		fps_d,
		sample_rate,
		channels,
		samples,
		flags,
	};
	if (!WriteAll(fd, reinterpret_cast<const uint8_t *>(&header), sizeof(header))) {
		return false;
	}
	if (payload_size == 0) {
		return true;
	}
	return WriteAll(fd, payload, payload_size);
}

bool SendTextPacket(int fd, PacketType packet_type, const std::string &text)
{
	return SendPacket(fd, packet_type, reinterpret_cast<const uint8_t *>(text.data()), text.size(), 0);
}

int BytesPerPixel(const NDIlib_video_frame_v2_t &video_frame)
{
	switch (video_frame.FourCC) {
		case NDIlib_FourCC_type_BGRA:
		case NDIlib_FourCC_type_BGRX:
		case NDIlib_FourCC_type_RGBA:
		case NDIlib_FourCC_type_RGBX:
			return 4;
		case NDIlib_FourCC_type_UYVY:
			return 2;
		default:
			return 0;
	}
}

bool PackVideoFrame(const NDIlib_video_frame_v2_t &video_frame, std::vector<uint8_t> *packed, uint32_t *packed_stride)
{
	const int bytes_per_pixel = BytesPerPixel(video_frame);
	if (bytes_per_pixel <= 0) {
		return false;
	}
	*packed_stride = static_cast<uint32_t>(video_frame.xres * bytes_per_pixel);
	const size_t total_size = static_cast<size_t>(*packed_stride) * static_cast<size_t>(video_frame.yres);
	packed->resize(total_size);
	const auto *src = static_cast<const uint8_t *>(video_frame.p_data);
	for (int y = 0; y < video_frame.yres; ++y) {
		std::memcpy(
			packed->data() + static_cast<size_t>(y) * (*packed_stride),
			src + static_cast<size_t>(y) * static_cast<size_t>(video_frame.line_stride_in_bytes),
			*packed_stride
		);
	}
	return true;
}

bool PackAudioFrameS16LE(const NDIlib_audio_frame_v3_t &audio_frame, std::vector<uint8_t> *packed)
{
	if (!audio_frame.p_data || audio_frame.no_channels <= 0 || audio_frame.no_samples <= 0 ||
	    audio_frame.channel_stride_in_bytes <= 0) {
		packed->clear();
		return true;
	}

	std::vector<int16_t> output(
		static_cast<size_t>(audio_frame.no_channels) * static_cast<size_t>(audio_frame.no_samples)
	);
	const auto *base = reinterpret_cast<const uint8_t *>(audio_frame.p_data);
	for (int ch = 0; ch < audio_frame.no_channels; ++ch) {
		const auto *plane = reinterpret_cast<const float *>(base + (ch * audio_frame.channel_stride_in_bytes));
		for (int sample = 0; sample < audio_frame.no_samples; ++sample) {
			const float clamped = std::clamp(plane[sample], -1.0f, 1.0f);
			const auto scaled = static_cast<int>(clamped * 32767.0f);
			output[static_cast<size_t>(sample) * static_cast<size_t>(audio_frame.no_channels) + static_cast<size_t>(ch)] =
				static_cast<int16_t>(std::clamp(
					scaled,
					static_cast<int>(std::numeric_limits<int16_t>::min()),
					static_cast<int>(std::numeric_limits<int16_t>::max())
				));
		}
	}

	packed->resize(output.size() * sizeof(int16_t));
	std::memcpy(packed->data(), output.data(), packed->size());
	return true;
}

int CreateListeningSocket(const std::string &socket_path)
{
	const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		return -1;
	}

	::unlink(socket_path.c_str());

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	if (socket_path.size() >= sizeof(addr.sun_path)) {
		::close(fd);
		return -1;
	}
	std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

	if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
		::close(fd);
		return -1;
	}
	if (::listen(fd, 1) < 0) {
		::close(fd);
		return -1;
	}
	return fd;
}

} // namespace

int main(int argc, char **argv)
{
	Args args;
	if (!ParseArgs(argc, argv, &args)) {
		Usage(argv[0]);
		return 2;
	}

	std::signal(SIGTERM, HandleSignal);
	std::signal(SIGINT, HandleSignal);

	const int listen_fd = CreateListeningSocket(args.socket_path);
	if (listen_fd < 0) {
		std::cerr << "ERROR: could not create Unix socket at " << args.socket_path << "\n";
		return 3;
	}

	std::cerr << "Waiting for AV bridge client on " << args.socket_path << "\n";
	const int client_fd = ::accept(listen_fd, nullptr, nullptr);
	if (client_fd < 0) {
		std::cerr << "ERROR: accept() failed for AV bridge socket\n";
		::close(listen_fd);
		::unlink(args.socket_path.c_str());
		return 3;
	}

	void *handle = LoadLibndiHandle();
	if (!handle) {
		SendTextPacket(client_fd, kError, std::string("failed to dlopen libndi.so.6: ") + dlerror());
		::close(client_fd);
		::close(listen_fd);
		::unlink(args.socket_path.c_str());
		return 3;
	}

	auto load_fn = reinterpret_cast<NDIlib_v6_load_>(dlsym(handle, "NDIlib_v6_load"));
	if (!load_fn) {
		SendTextPacket(client_fd, kError, std::string("failed to resolve NDIlib_v6_load: ") + dlerror());
		dlclose(handle);
		::close(client_fd);
		::close(listen_fd);
		::unlink(args.socket_path.c_str());
		return 3;
	}

	const NDIlib_v6 *ndi = load_fn();
	if (!ndi || !ndi->initialize()) {
		SendTextPacket(client_fd, kError, "failed to initialize NDI");
		dlclose(handle);
		::close(client_fd);
		::close(listen_fd);
		::unlink(args.socket_path.c_str());
		return 3;
	}

	NDIlib_recv_create_v3_t recv_desc = {};
	recv_desc.allow_video_fields = true;
	recv_desc.bandwidth = NDIlib_recv_bandwidth_highest;
	recv_desc.color_format = args.use_uyvy
		? NDIlib_recv_color_format_UYVY_BGRA   // UYVY preferred (2 bpp), BGRA fallback
		: NDIlib_recv_color_format_BGRX_BGRA;   // BGRX/BGRA (4 bpp)
	recv_desc.p_ndi_recv_name = args.recv_name.c_str();
	recv_desc.source_to_connect_to.p_ndi_name = args.ndi_name.c_str();
	recv_desc.source_to_connect_to.p_url_address = args.url_address.empty() ? nullptr : args.url_address.c_str();

	NDIlib_recv_instance_t receiver = ndi->recv_create_v3(&recv_desc);
	if (!receiver) {
		SendTextPacket(client_fd, kError, "ndi->recv_create_v3 returned null");
		ndi->destroy();
		dlclose(handle);
		::close(client_fd);
		::close(listen_fd);
		::unlink(args.socket_path.c_str());
		return 3;
	}

	NDIlib_metadata_frame_t hw_metadata = {};
	hw_metadata.length = 0;
	hw_metadata.timecode = NDIlib_send_timecode_synthesize;
	hw_metadata.p_data = const_cast<char *>("<ndi_video_codec type=\"hardware\"/>");
	SendTextPacket(
		client_fd,
		kStatus,
		std::string("Initial hardware accel metadata request ok=") +
			(ndi->recv_send_metadata(receiver, &hw_metadata) ? "true" : "false")
	);

	NDIlib_tally_t tally = {};
	tally.on_preview = true;
	tally.on_program = true;
	SendTextPacket(
		client_fd,
		kStatus,
		std::string("Initial tally request ok=") +
			(ndi->recv_set_tally(receiver, &tally) ? "true" : "false")
	);

	bool first_frame_seen = false;
	bool hw_metadata_retried = false;
	uint64_t video_frames_captured = 0;
	uint64_t video_frames_dropped = 0;
	uint64_t audio_frames_captured = 0;
	uint64_t video_frames_sent = 0;
	uint64_t audio_frames_sent = 0;
	// Frame-rate limiting: minimum interval between sent video frames
	const double min_frame_interval_us = args.max_fps > 0
		? (1'000'000.0 / static_cast<double>(args.max_fps))
		: 0.0;
	auto last_video_sent = Clock::now() - std::chrono::seconds(1);
	const auto startup_deadline =
		Clock::now() + std::chrono::milliseconds(static_cast<int>(args.startup_timeout_seconds * 1000.0));

	while (g_running.load()) {
		NDIlib_video_frame_v2_t video_frame = {};
		NDIlib_audio_frame_v3_t audio_frame = {};
		NDIlib_metadata_frame_t metadata_frame = {};

		const auto frame_type = ndi->recv_capture_v3(receiver, &video_frame, &audio_frame, &metadata_frame, 250);

		if (!first_frame_seen && Clock::now() >= startup_deadline && ndi->recv_get_no_connections(receiver) == 0) {
			SendTextPacket(client_fd, kError, "no NDI connection before startup timeout");
			break;
		}

		if (!hw_metadata_retried && ndi->recv_get_no_connections(receiver) > 0) {
			SendTextPacket(
				client_fd,
				kStatus,
				std::string("Hardware accel metadata retry after connect ok=") +
					(ndi->recv_send_metadata(receiver, &hw_metadata) ? "true" : "false")
			);
			hw_metadata_retried = true;
		}

		switch (frame_type) {
			case NDIlib_frame_type_video: {
				first_frame_seen = true;
				++video_frames_captured;
				// Frame-rate limiting: skip if too soon since last sent frame
				if (min_frame_interval_us > 0.0) {
					const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
						Clock::now() - last_video_sent).count();
					if (static_cast<double>(elapsed_us) < min_frame_interval_us) {
						++video_frames_dropped;
						ndi->recv_free_video_v2(receiver, &video_frame);
						break;
					}
				}
				std::vector<uint8_t> packed;
				uint32_t packed_stride = 0;
				if (!PackVideoFrame(video_frame, &packed, &packed_stride)) {
					SendTextPacket(client_fd, kError, "unsupported video FourCC from NDI receiver");
					ndi->recv_free_video_v2(receiver, &video_frame);
					g_running.store(false);
					break;
				}
				if (!SendPacket(
						client_fd,
						kVideo,
						packed.data(),
						packed.size(),
						video_frame.timestamp,
						static_cast<uint32_t>(video_frame.xres),
						static_cast<uint32_t>(video_frame.yres),
						packed_stride,
						static_cast<uint32_t>(video_frame.FourCC),
						static_cast<uint32_t>(video_frame.frame_rate_N),
						static_cast<uint32_t>(video_frame.frame_rate_D)
					)) {
					std::cerr << "ERROR: failed to send video packet to AV bridge client\n";
					g_running.store(false);
				} else {
					++video_frames_sent;
					last_video_sent = Clock::now();
				}
				ndi->recv_free_video_v2(receiver, &video_frame);
				break;
			}

			case NDIlib_frame_type_audio: {
				first_frame_seen = true;
				++audio_frames_captured;
				std::vector<uint8_t> packed;
				if (!PackAudioFrameS16LE(audio_frame, &packed)) {
					SendTextPacket(client_fd, kError, "could not convert NDI audio to S16LE");
					ndi->recv_free_audio_v3(receiver, &audio_frame);
					g_running.store(false);
					break;
				}
				if (!SendPacket(
						client_fd,
						kAudio,
						packed.data(),
						packed.size(),
						audio_frame.timestamp,
						0,
						0,
						static_cast<uint32_t>(audio_frame.no_channels * sizeof(int16_t)),
						0,
						0,
						0,
						static_cast<uint32_t>(audio_frame.sample_rate),
						static_cast<uint32_t>(audio_frame.no_channels),
						static_cast<uint32_t>(audio_frame.no_samples)
					)) {
					std::cerr << "ERROR: failed to send audio packet to AV bridge client\n";
					g_running.store(false);
				} else {
					++audio_frames_sent;
				}
				ndi->recv_free_audio_v3(receiver, &audio_frame);
				break;
			}

			case NDIlib_frame_type_metadata:
				SendTextPacket(client_fd, kStatus, "metadata frame received");
				ndi->recv_free_metadata(receiver, &metadata_frame);
				break;

			case NDIlib_frame_type_status_change:
				SendTextPacket(client_fd, kStatus, "receiver status changed");
				break;

			case NDIlib_frame_type_error:
				SendTextPacket(client_fd, kError, "NDI receive returned frame_type_error");
				g_running.store(false);
				break;

			default:
				break;
		}
	}

	SendPacket(client_fd, kEos, nullptr, 0, 0);
	std::cerr << "AV bridge exiting with video_frames_captured=" << video_frames_captured
		  << " audio_frames_captured=" << audio_frames_captured
		  << " video_frames_sent=" << video_frames_sent
		  << " audio_frames_sent=" << audio_frames_sent << "\n";
	ndi->recv_destroy(receiver);
	ndi->destroy();
	dlclose(handle);
	::close(client_fd);
	::close(listen_fd);
	::unlink(args.socket_path.c_str());
	return 0;
}
