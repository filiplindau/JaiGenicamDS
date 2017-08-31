#ifndef JaiGenicamCameraControl_H
#define JaiGenicamCameraControl_H
//#include "JaiGenicamConnection.h"
#include <string>
#include <mutex>
#include <atomic>
#include <queue>
#include <map>
#include <functional>
#include <thread>
#include <jai_factory.h>
//#include <tango.h>


#include "Signal.h"

namespace JaiGenicamCameraControl_ns
{	
	#define NODE_NAME_WIDTH         (int8_t*)"Width"
	#define NODE_NAME_HEIGHT        (int8_t*)"Height"
	#define NODE_NAME_PIXELFORMAT   (int8_t*)"PixelFormat"
	#define NODE_NAME_GAIN          (int8_t*)"GainRaw"
	#define NODE_NAME_ACQSTART      (int8_t*)"AcquisitionStart"
	#define NODE_NAME_ACQSTOP       (int8_t*)"AcquisitionStop"	

	// Data structures
	template <typename T>
	struct GenicamNode
	{
		std::string	name;
		std::string	type;
		std::string	unit;
		std::string	description;
		T		value;
		T		min_value;
		T		max_value;
		bool	valid;
	};

	struct GenicamEnumEntry
	{
		std::string name;
		int64_t value;
	};


	struct GenicamGenericNode
	{
		std::string	name;
		J_NODE_TYPE	type;
		std::string	unit;
		std::string	description;
		double	value_d;
		double	min_value_d;
		double	max_value_d;
		int64_t	value_i;
		int64_t	min_value_i;
		int64_t	max_value_i;
		std::vector<std::string> enum_names;
		std::map<std::string, int64_t> enum_entry_map;
		std::map<int64_t, std::string> enum_value_map;
		std::string	value_s;
		bool	valid;
	};

	enum CameraState 
	{
		UNKNOWN_STATE,
		DISCONNECTED_STATE,
		INIT_STATE,
		IDLE_STATE,
		RUNNING_STATE,
		FAULT_STATE,
		NO_STATE
	};

	struct GenicamErrorStruct
	{
		CameraState calling_state;
		std::string state_string;
		std::string calling_function;
		std::string camera_function;
		std::string error_message;
		J_STATUS_TYPE retval;
	};


	struct CameraStateText {
		static const char* enum_text[];
	};

	// const char* CameraStateText::enum_text[] = {"UNKNOWN", "DISCONNECTED", "INIT", "IDLE", "RUNNING", "FAULT"};
	// const char* CameraStateText::enum_text[];

	enum CameraCommand
	{
		INIT_CMD,
		START_CAPTURE_CMD,
		STOP_CAPTURE_CMD,
		DISCONNECT_CMD,
		CONNECT_CMD,
		GET_GAIN_CMD,
		GET_EXPOSURE_CMD,
		SET_GAIN_CMD,
		SET_EXPOSURE_CMD,
		GET_NODE_CMD,
		SET_NODE_CMD,
		NO_CMD
	};

	enum CameraCommandTypeEnum
	{
		INT,
		FLOAT,
		STRING,
		ENUM_INT,
		ENUM_STRING
	};
	struct CameraCommandData
	{
		CameraCommand	command;
		std::string		name_string;
		int				data_int;
		float			data_float;
		std::string		data_string;
		CameraCommandTypeEnum type;
	};

	class JaiGenicamCameraControl
	{
	public:
		JaiGenicamCameraControl(std::string camera_serial)
		{
			// Init state map. Each state has a handler method associated with it that gets executed when switching to that state.
			state_map[CameraState::UNKNOWN_STATE] = [this](CameraState prev_state) -> CameraState {return unknown_handler(prev_state);};
			state_map[CameraState::DISCONNECTED_STATE] = [this](CameraState prev_state) -> CameraState {return disconnected_handler(prev_state);};
			state_map[CameraState::INIT_STATE] = [this](CameraState prev_state) -> CameraState {return init_handler(prev_state);};
			state_map[CameraState::IDLE_STATE] = [this](CameraState prev_state) -> CameraState {return idle_handler(prev_state);};
			state_map[CameraState::RUNNING_STATE] = [this](CameraState prev_state) -> CameraState {return running_handler(prev_state);};
			state_map[CameraState::FAULT_STATE] = [this](CameraState prev_state) -> CameraState {return fault_handler(prev_state);};	

			this->state_enum_text.push_back("UNKNOWN");
			this->state_enum_text.push_back("DISCONNECTED");
			this->state_enum_text.push_back("INIT");
			this->state_enum_text.push_back("IDLE");
			this->state_enum_text.push_back("RUNNING");
			this->state_enum_text.push_back("FAULT");

			// Camera serial number
			this->camera_serial = camera_serial;

			// Flag to terminate the state loop
			this->stop_state_flag = false;

			// Stick status message
			this->sticky_status_message = "";

			// Init handles
			this->camera_handle = NULL;
			this->capture_thread_handle = NULL;
			this->factory_handle = NULL;
			this->image_buffer.pImageBuffer = NULL;

			// Init frame counter
			this->frame_counter = 0;
			this->fps_vector.clear();

			// Start state handler thread
			state_handler_thread = std::thread(&JaiGenicamCameraControl::state_handler_dispatcher, this);
		}

		~JaiGenicamCameraControl(void);

		// Client camera commands
		int start_capture(void);
		int stop_capture(void);
		int disconnect(void);
		int connect(void);
		int get_node_value(std::string name, double &value);
		int get_node_value(std::string name, int64_t &value);
		int get_node(std::string node_name, GenicamGenericNode &node);
		int set_node(GenicamGenericNode node);
		int set_node_value(std::string name, double value);
		int set_node_value(std::string name, int64_t value);
		int set_node_value(std::string name, std::string value);
		int get_node_type(std::string name, std::string &type);
		int get_node_info(std::string name, GenicamGenericNode &generic_node);

		int get_image(uint32_t &width, uint32_t &height, uint16_t* image_p);
		int get_image_info(J_tIMAGE_INFO &aq_image_info);
		int get_framecounter(int64_t &value);
		int get_framerate(double &value);

		int JaiGenicamCameraControl::get_camera_list(std::vector<std::string> &camera_list);

		CameraState get_state(void);
		void send_command(CameraCommandData command_data);		

		Signal<int>				image_ready_signal;
		Signal<CameraState>		state_changed_signal;
		Signal<std::string>			status_message_signal;
		Signal<GenicamErrorStruct>	error_signal;
		Signal<CameraCommandData>	command_return_signal;

	private:
		// Command queue and mutexes
		std::queue<CameraCommandData>	command_queue;
		std::mutex						queue_mutex;
		std::mutex						camera_mutex;
		std::mutex						error_mutex;
		CameraState						state;

		CameraCommandData check_commands();

		// State handlers
		std::atomic<bool>	stop_state_flag;
		std::thread			state_handler_thread;
		void				state_handler_dispatcher();

		CameraState	unknown_handler(CameraState prev_state);
		CameraState	init_handler(CameraState prev_state);
		CameraState	disconnected_handler(CameraState prev_state);
		CameraState	idle_handler(CameraState prev_state);
		CameraState	running_handler(CameraState prev_state);
		CameraState	fault_handler(CameraState prev_state);

		std::map<CameraState, std::function<CameraState(CameraState)>>	state_map;
		std::vector<std::string> state_enum_text;

		// Handles
		FACTORY_HANDLE  factory_handle;     // Factory Handle
		CAM_HANDLE      camera_handle;      // Camera Handle
		THRD_HANDLE		capture_thread_handle;
		
		std::string		camera_serial;
		int8_t          camera_id_s[J_CAMERA_ID_SIZE];

		// Image stuff
		J_tIMAGE_INFO		image_buffer;
		int64_t				frame_counter;
		std::vector<double> fps_vector;

		// Nodes
		std::mutex	node_mutex;
		std::map<std::string, GenicamGenericNode> node_map;
		std::string				exposuretime_node_name;
		GenicamNode<double>		exposuretime_node;
		GenicamNode<uint64_t>	gain_node;
		GenicamNode<uint64_t>	framerate_node;
		GenicamNode<uint64_t>	imagewidth_node;
		GenicamNode<uint64_t>	imageheight_node;
		GenicamNode<uint64_t>	pixelformat_node;

		// Error handling
		GenicamErrorStruct	error_data;
		std::string			sticky_status_message;	// Extra status message appended at end of every status message to notify sticky stuff
		std::string JaiGenicamCameraControl::get_error_string(J_STATUS_TYPE error);
		int JaiGenicamCameraControl::set_error_data(std::string calling_function, std::string camera_function, std::string error_message, int retval, CameraState calling_state, bool emit_signal);
		int JaiGenicamCameraControl::set_error_state(CameraState state);
		int JaiGenicamCameraControl::set_sticky_status_message(std::string sticky_status_msg, bool append);
		int JaiGenicamCameraControl::emit_status_message(std::string status_msg);
		GenicamErrorStruct JaiGenicamCameraControl::get_error_data();
		int JaiGenicamCameraControl::clear_error_data();

		bool is_ipaddress(std::string ip_string);

		// Camera access
		void __stdcall JaiGenicamCameraControl::capture_stream_callback(J_tIMAGE_INFO * aq_imageinfo_p);
		int start_camera_acquisition();
		int stop_camera_acquisition();
		int open_camera();
		int close_camera();
		int JaiGenicamCameraControl::find_camera(std::string serial, int8_t* camera_id_s_p);
		int open_factory();
		int close_factory();
		int populate_node_map();
//		GenicamGenericNode generate_genericnode_from_name(std::string node_name);
		int generate_genericnode_from_name(std::string node_name, GenicamGenericNode &generic_node_result);
		int get_node_from_camera(std::string name, GenicamGenericNode &generic_node);
		int set_node_to_camera(GenicamGenericNode generic_node);
		int JaiGenicamCameraControl::disable_auto_nodes();
	};
}

#endif /* JaiGenicamCameraControl_H */