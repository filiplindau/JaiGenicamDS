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
	#define NODE_NAME_PAYLOADSIZE   (int8_t*)"PayloadSize"
	#define NODE_NAME_GAIN          (int8_t*)"GainRaw"
	#define NODE_NAME_ACQSTART      (int8_t*)"AcquisitionStart"
	#define NODE_NAME_ACQSTOP       (int8_t*)"AcquisitionStop"
	#define NODE_NAME_RESET			(int8_t*)"DeviceReset"

	#define USE_STREAMTHREAD		1	// Determines if we use custom stream thread function (1) or built-in J_image_openstream (0)
	#define	NUM_OF_BUFFERS	        5

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

	/** Enum of available commands to send to the control thread.
	*/
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
		UPDATE_NODE_CMD,
		RESET_CAMERA_CMD,
		
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

	/** Struct for sending commands to the camera control thread.
	*/
	struct CameraCommandData
	{
		CameraCommand	command;
		std::string		name_string;
		int				data_int;
		float			data_float;
		std::string		data_string;
		CameraCommandTypeEnum type;
	};


	/** The camera control class.

	This will start a state machine for camera control. Direct camera access is not available from the outside, it is handled in 
	the state handler methods in its own thread. Communication is sent with a command queue by using public methods. The state handler 
	is started in the constructor. It takes as an argument the serial number of the camera you wish to connect to.
	
	Some care has been taken to make the whole class thread safe. Camera access is protected with the camera_mutex, queue access 
	has the queue_mutex. There is also an error_mutex for error message handling.

	State changes, status changes, new image ready, error message, node updated, and command return are signalled with Signals
	(see http://simmesimme.github.io/tutorials/2015/09/20/signal-slot). To receive the updates a client connects to a signal with e.g.
	my_camera_control.image_ready_signal.connect_member(this, &MyClientClass::update_image);


	The state transitions are:
	--start--
	UNKNOWN STATE: 
	Start up the JaiGenicamFactory. Find the desired camera on the bus. 
	-> UNKNOWN: If the camera is not found
	-> INIT: Camera found
	
	INIT STATE:
	Open the camera and populate the node map. Disable nodes with the name "auto" in them. They are annoying when controlling the camera.
	-> FAULT: Error detected
	-> IDLE: Everythink ok

	IDLE STATE:
	Stop acquisition thread if running. Go in a loop, checking the command queue for commands.
	-> FAULT: Error detected
	-> IDLE: No state transition from command
	-> RUNNING: START_CAPTURE_CMD
	-> DISCONNECTED: DISCONNECT_CMD
	-> INIT: INIT_CMD

	RUNNING STATE:
	Start acquisition thread if not running. Go in a loop, checking the command queue for commands.
	-> FAULT: Error detected
	-> IDLE: STOP_CAPTURE_CMD
	-> RUNNING: No state transition from command
	-> DISCONNECTED: DISCONNECT_CMD
	-> INIT: INIT_CMD

	DISCONNECTED STATE:
	Stop acquisition thread if running. Disconnect from camera, freeing it for other applications. 
	Go in a loop, checking the command queue for commands.
	-> IDLE: CONNECT_CMD
	-> RUNNING: START_CAPTURE_CMD
	-> DISCONNECTED: No state transition from command
	-> INIT: INIT_CMD

	FAULT STATE:
	An error has occurred. Try to identify the error through the error_data struct. This is done in a long if-else construct.
	-> Previous state: If the error can be rectified
	-> UNKNOWN: Default state if the error correction is unsuccessful
	

	After connecting to the camera a node map is created. It consists of "generic node" structs, which has fields for int, float, and
	string nodes. Only feature nodes are stored (not subfeatures such as GainMax). This map is then used to retrieve node information quickly.
	A problem is that sometimes a node is not readable or writeable. These errors are "ignored", i.e. not sent to the fault handler state.

	When starting the image acquisition (RUNNING state), a new thread is started. If the define USE_STREAMTHREAD = 0 the JaiGenicam convenience 
	functions J_Camera_CreateDatastream etc. are used. This was found to be problematic when running several cameras simultaneously. 
	So setting USE_STREAMTHREAD = 1 will use the J_Datastream_xxx functions instead with manual buffer management etc. This seems to
	work better.

	TODO: Sometimes when running several cameras the camera acquisition stalls, getting J_COND_WAIT_TIMEOUT events. It helps to increase 
	the packetdelay node. Should put in code to restart the acquisition when it stalls.
	*/
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
			this->stream_event_handle = NULL;
			this->stream_handle = NULL;
//			this->stream_thread_handle = NULL;
			this->event_condition_handle = NULL;
			this->stream_started = false;
			this->enable_thread = false;

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
		int reset(void);
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
		int get_node_map_list(std::vector<std::string> &node_map_list);

		int get_image(uint32_t &width, uint32_t &height, uint16_t* image_p);
		int get_image_info(J_tIMAGE_INFO &aq_image_info);
		int get_framecounter(int64_t &value);
		int get_framerate(double &value);

		int update_nodeinfo(std::string node_name);

		int JaiGenicamCameraControl::get_camera_list(std::vector<std::string> &camera_list);

		CameraState get_state(void);
		void send_command(CameraCommandData command_data);		

		Signal<int>				image_ready_signal;
		Signal<CameraState>		state_changed_signal;
		Signal<std::string>			status_message_signal;
		Signal<GenicamErrorStruct>	error_signal;
		Signal<CameraCommandData>	command_return_signal;
		Signal<GenicamGenericNode>	update_node_signal;

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
//		HANDLE			stream_thread_handle;
		std::thread		stream_thread_handle;
		STREAM_HANDLE	stream_handle;
		HANDLE			stream_event_handle;
		HANDLE			event_condition_handle;
		
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
		int reset_camera();
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
		int update_nodemap_nodeinfo(std::string node_name);

		// Data stream functions
		bool enable_thread;
		bool stream_started;
		uint32_t	valid_buffers;
		uint8_t*    aq_buffer[NUM_OF_BUFFERS];
		BUF_HANDLE  aq_buffer_id[NUM_OF_BUFFERS];
		int start_datastream();
		int stop_datastream();
//		int create_stream_thread(uint32_t channel_id, uint32_t buffersize);
		int terminate_stream_thread(void);   // Terminate the image acquisition thread
		void stream_process(void);           // The actual image acquisition thread
//		void terminate_thread(void);         
		void wait_for_thread_to_terminate(void);
		void close_thread_handle(void);
		int prepare_buffer(int buffersize, uint32_t &valid_buffers);
		int unprepare_buffer(void);
	};
}

#endif /* JaiGenicamCameraControl_H */