#pragma once
#include <tango.h>
#include <jai_factory.h>
#include <mutex>
#include <thread>
#include <map>


/* State machine for controlling genicam cameras.

It starts disconnected (doing nothing)
Transitions:
DisconnectedState [connect] -> InitState
InitState -> ConnectedState (after completion)
InitState [disconnect] -> DisconnectedState
ConnectedState [start]-> RunningState
ConnectedState [disconnect] -> DisconnectedState
ConnectedState [init]-> InitState
RunningState [stop]-> ConnectedState
RunningState [disconnect] -> DisconnectedState
RunningState [init]-> InitState

*/
namespace JaiGenicamDS_ns
	{
		class JaiGenicamDS;
	}

namespace JaiGenicamConnection_ns
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
		string	name;
		string	type;
		string	unit;
		string	description;
		T		value;
		T		min_value;
		T		max_value;
		bool	valid;
	};

	struct GenicamEnumEntry
	{
		string name;
		int64_t value;
	};


	struct GenicamGenericNode
	{
		string	name;
		string	type;
		string	unit;
		string	description;
		double	value_d;
		double	min_value_d;
		double	max_value_d;
		int64_t	value_i;
		int64_t	min_value_i;
		int64_t	max_value_i;
		vector<string> enum_names;
		map<string, int64_t> enum_entry_map;
		map<int64_t, string> enum_value_map;
		bool	valid;
	};


	// Forward declarations
	class JaiGenicamState;
	


	class JaiGenicamConnection
	{
	public:
		JaiGenicamConnection(::JaiGenicamDS_ns::JaiGenicamDS * tango_ds_class, std::string camera_serial);
		~JaiGenicamConnection(void);

		int get_image(uint32_t* width_p, uint32_t* height_p, uint16_t* image_p);
		int get_image_info(J_tIMAGE_INFO* aq_image_info_p);
		int get_framecounter(int64_t* value_p);
		int get_gain(double* value_p);
		int get_framerate(double* value_p);
/*		void set_gain();
		void get_exposuretime();
		void set_exposuretime();
		void get_framerate();
		void set_framerate();
		void get_externaltrigger();
		void set_externaltrigger();
		void get_framecounter();*/

		int get_node_value(std::string name, double* value_p);
		int get_node_value(std::string name, int64_t* value_p);
		int set_node_value(std::string name, double value);

		int get_node_type(std::string name, std::string* type);

		int get_node_info(std::string name, GenicamGenericNode* generic_node_p);

		void start_capture();
		void stop_capture();
		void connect();
		void disconnect();

	private:
		// Functions
		void run();

		template<typename T>
		int get_node(GenicamNode<T>* node);

		void __stdcall capture_stream_callback(J_tIMAGE_INFO * aq_imageinfo_p);

		std::string JaiGenicamConnection::get_error_string(J_STATUS_TYPE error);

	private:
		// State classes, variables
		friend class JaiGenicamState;
		friend class JaiGenicamDisconnectedState;
		friend class JaiGenicamInitState;
		friend class JaiGenicamConnectedState;
		friend class JaiGenicamFaultState;
		friend class JaiGenicamRunningState;
		void close_camera();
		void close_factory();
		void change_state( JaiGenicamState* );
		JaiGenicamState* jai_genicam_state;		

	private:
		// Variables
		Tango::DevState tango_state;
		::JaiGenicamDS_ns::JaiGenicamDS* tango_ds_class;

		std::mutex camera_mtx;

		// Handles
		FACTORY_HANDLE  factory_handle;     // Factory Handle
		CAM_HANDLE      camera_handle;      // Camera Handle
		THRD_HANDLE		capture_thread_handle;
		
		std::string		camera_serial;

		// Image stuff
		J_tIMAGE_INFO	image_buffer;
		int64_t			frame_counter;
		std::vector<double> fps_vector;

		// Nodes
		GenicamNode<double> exposuretime_node;
		GenicamNode<uint64_t> gain_node;
		GenicamNode<uint64_t> framerate_node;
		GenicamNode<uint64_t> imagewidth_node;
		GenicamNode<uint64_t> imageheight_node;
		GenicamNode<uint64_t> pixelformat_node;
		
	};


	/////////////////////////////////////////////////////////////////////////////////

	class JaiGenicamState 
	{
	public:	
		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );
		virtual int close_camera(JaiGenicamConnection* wrapper);
		virtual int close_factory(JaiGenicamConnection* wrapper);
		virtual int connect(JaiGenicamConnection* wrapper);
		virtual int disconnect(JaiGenicamConnection* wrapper);
		virtual int start_capture(JaiGenicamConnection* wrapper);
		virtual int stop_capture(JaiGenicamConnection* wrapper);

		virtual int get_gain(JaiGenicamConnection* wrapper, double* value_p);
		virtual int get_image(JaiGenicamConnection* wrapper, uint32_t* width_p, uint32_t* height_p, uint16_t* image_p);
		virtual int get_image_info(JaiGenicamConnection* wrapper, J_tIMAGE_INFO* aq_image_info_p);
		virtual int get_framecounter(JaiGenicamConnection* wrapper, int64_t* value_p);
		virtual int JaiGenicamState::get_framerate(JaiGenicamConnection* wrapper, double* value_p);

		virtual int get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p);
		virtual int get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p);
		virtual int set_node_value(JaiGenicamConnection* wrapper, std::string name, double value);

		virtual int get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type);

		virtual int get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p);

		
		template<typename T>
		int get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node);
		

	protected:
		void change_state( JaiGenicamConnection*, JaiGenicamState* );
	
	};

	// JaiGenicamDisconnectedState
	class JaiGenicamDisconnectedState : JaiGenicamState
	{
	public:
		static JaiGenicamState* Instance()
		{
			if (_instance == NULL)
			{
				_instance = new JaiGenicamDisconnectedState;
			}
			return _instance;
		};
		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );
		virtual int connect( JaiGenicamConnection* wrapper );
		virtual int close_camera(JaiGenicamConnection* wrapper);
		virtual int close_factory(JaiGenicamConnection* wrapper);		

	protected:
		JaiGenicamDisconnectedState() {} ;

	private:
		static JaiGenicamDisconnectedState* _instance;
	};

	// JaiGenicamInitState
	class JaiGenicamInitState : JaiGenicamState
	{
	public:
		static JaiGenicamState* Instance()
		{
			if (_instance == NULL)
			{
				_instance = new JaiGenicamInitState;
			}
			return _instance;
		};

		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );

	protected:
		JaiGenicamInitState() {} ;

	private:
		static JaiGenicamInitState* _instance;

	};

	// JaiGenicamConnectedState
	class JaiGenicamConnectedState : JaiGenicamState
	{
	public:
		static JaiGenicamState* Instance()
		{
			if (_instance == NULL)
			{
				_instance = new JaiGenicamConnectedState;
			}
			return _instance;
		};

		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );

		int start_capture(JaiGenicamConnection* wrapper);
		int stop_capture(JaiGenicamConnection* wrapper);

//		virtual double get_gain(JaiGenicamConnection* wrapper);
		
		template<typename T>
		int get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node);

		int get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p);
		int get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p);
		int set_node_value(JaiGenicamConnection* wrapper, std::string name, double value);

		int get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type);

		int get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p);

	protected:
		JaiGenicamConnectedState() {} ;

	private:
		static JaiGenicamConnectedState* _instance;

	};

	// JaiGenicamFaultState
	class JaiGenicamFaultState : JaiGenicamState
	{
	public:
		static JaiGenicamState* Instance()
		{
			if (_instance == NULL)
			{
				_instance = new JaiGenicamFaultState;
			}
			return _instance;
		};

		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );

	protected:
		JaiGenicamFaultState() {} ;

	private:
		static JaiGenicamFaultState* _instance;
	};

	// JaiGenicamRunningState
	class JaiGenicamRunningState : JaiGenicamState
	{
	public:
		static JaiGenicamState* Instance()
		{
			if (_instance == NULL)
			{
				_instance = new JaiGenicamRunningState;
			}
			return _instance;
		};

		virtual int enter( JaiGenicamConnection* wrapper );
		virtual int run( JaiGenicamConnection* wrapper );
		int start_capture(JaiGenicamConnection* wrapper);
		int stop_capture(JaiGenicamConnection* wrapper);
		int connect(JaiGenicamConnection* wrapper);

		template<typename T>
		int get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node);

		int get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p);
		int get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p);
		int set_node_value(JaiGenicamConnection* wrapper, std::string name, double value);

		int get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type);

		int get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p);

	protected:
		JaiGenicamRunningState() {};

		int execute_start_capture(JaiGenicamConnection* wrapper);

	private:
		static JaiGenicamRunningState* _instance;
	};

} // namespace JaiGenicamConnection_ns