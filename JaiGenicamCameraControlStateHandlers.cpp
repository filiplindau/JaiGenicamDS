#include "JaiGenicamCameraControl.h"
#include <set>
#include <stack>

using namespace JaiGenicamCameraControl_ns;



/** State handler dispatcher. Runs the state handler loop and selects the correct handler function depending on state.
The previous state is sent along to the handler function to enable history dependent behaviour. 

The loop is exited
*/
void JaiGenicamCameraControl::state_handler_dispatcher()
{
	CameraState prev_state = CameraState::UNKNOWN_STATE;
	CameraState next_state = CameraState::UNKNOWN_STATE;
	CameraState new_state = CameraState::UNKNOWN_STATE;
	while (this->stop_state_flag != true)
	{
		std::cout << std::endl << "====================================================================" << std::endl;
		std::cout << "Entering camera state " << this->state_enum_text[next_state] << std::endl << std::endl;
		std::unique_lock<std::mutex> lock(this->queue_mutex, std::defer_lock);
		// Lock the queue mutex before changing the internal state variable:
		lock.lock();
		this->state = next_state;
		lock.unlock();
		// Emit state change signal:
		this->state_changed_signal.emit(next_state);
		// Execute handler function for next state (stored in a map):
		new_state = this->state_map[next_state](prev_state);
		prev_state = next_state;
		next_state = new_state;		
	};
};


/** Handler function for the UNKNOWN_STATE. It checks which cameras are available and if the one we are looking for is among them.
If it is, the we go to INIT_STATE.
Otherwise wait and look again.
*/
CameraState JaiGenicamCameraControl::unknown_handler(CameraState prev_state)
{
	J_STATUS_TYPE   retval;
	bool8_t         has_changed;
	uint32_t        n_cameras;
	int8_t          camera_id_s[J_CAMERA_ID_SIZE];
	uint32_t        size;

	std::stringstream status_stream;
	std::stringstream err_msg;
	GenicamErrorStruct error_data;

	this->set_sticky_status_message("", false);
	status_stream << "Finding camera" << std::endl;
	this->emit_status_message(status_stream.str());

	retval = this->open_factory();
	if (retval != 0)
	{
		err_msg << "Could not open factory!" << this->get_error_string(retval); 
		std::cout << err_msg.str() << std::endl;
		this->emit_status_message(err_msg.str());
		error_data = this->get_error_data();
//		this->error_signal.emit(error_data);

		return CameraState::FAULT_STATE;
	}
	status_stream << "Camera factory open" << std::endl;
//	this->emit_status_message(status_stream.str());	

	retval = this->find_camera(this->camera_serial, camera_id_s);
	if (retval != 0)
	{
		this->set_error_state(CameraState::UNKNOWN_STATE);
		std::stringstream err_stream;
		err_stream << "unknown_handler: Error find_camera, returned " << this->get_error_string(retval) << std::endl;
		return CameraState::FAULT_STATE;
	};
	std::copy(std::begin(camera_id_s), std::end(camera_id_s), std::begin(this->camera_id_s));

	return CameraState::INIT_STATE;
}; // JaiGenicamCameraControl::unknown_handler


/** Handler function for the INIT_STATE. It connects to the camera and downloads basic node information.
*/
CameraState JaiGenicamCameraControl::init_handler(CameraState prev_state)
{
	J_STATUS_TYPE   retval;
	bool8_t         has_changed;
	uint32_t        n_cameras;
	uint32_t        size;
	std::stringstream error_stream;
	GenicamErrorStruct error_data;
	std::stringstream status_stream;

	status_stream << "Init_handler: Initializing camera" << std::endl;
	this->emit_status_message(status_stream.str());

	retval = this->open_camera();
	if (retval != 0)
	{
		error_stream <<  "Init_handler: Could not open camera!" << this->get_error_string(retval) << std::endl;
//		std::cout << error_stream.str();
		status_stream << error_stream.str();
//		this->emit_status_message(status_stream.str());

		error_stream << "init_handler: state " << error_data.calling_state << " ,calling function " << error_data.calling_function << ", camera function " << error_data.camera_function << std::endl;
		std::cout << error_stream.str();
//		this->error_signal.emit(error_data);
		this->set_error_state(CameraState::INIT_STATE);
		return CameraState::FAULT_STATE;
	}
	status_stream << "Init_handler: Camera open" << std::endl;
	this->emit_status_message(status_stream.str());

	retval = this->populate_node_map();
	if (retval != 0)
	{
		error_stream <<  "Init_handler: Could not populate node map!" << this->get_error_string(retval) << std::endl;
//		std::cout << error_stream.str();
		status_stream << error_stream.str();
		this->set_error_state(CameraState::INIT_STATE);
		return CameraState::FAULT_STATE;
	};
	status_stream << "Init_handler: Node map populated" << std::endl;
	this->emit_status_message(status_stream.str());

	retval = this->disable_auto_nodes();
	if (retval != 0)
	{
		error_stream <<  "Init_handler: Could not disable auto nodes!" << this->get_error_string(retval) << std::endl;
//		std::cout << error_stream.str();
		status_stream << error_stream.str();
//		this->emit_status_message(status_stream.str());

		this->set_error_state(CameraState::INIT_STATE);
		return CameraState::FAULT_STATE;
	};
	status_stream << "Init_handler: Auto nodes disabled" << std::endl;
	this->emit_status_message(status_stream.str());

	return CameraState::IDLE_STATE;
}; // JaiGenicamCameraControl::init_handler



/** Handler function for the DISCONNECTED_STATE. We are disconnected from the camera. It waits in a loop checking commands periodically. 
*/
CameraState JaiGenicamCameraControl::disconnected_handler(CameraState prev_state)
{
	// Lock that can be re-locked as needed for queue access:
	std::unique_lock<std::mutex> lock(this->queue_mutex, std::defer_lock);
	CameraCommandData cmd_data;		// Variable for storing incoming commands
	GenicamGenericNode generic_node; // Variable for setting nodes
	int retval;
	CameraState new_state = CameraState::DISCONNECTED_STATE;	// Variable for storing shift in state
	std::set<CameraState> handled_states;				// Set of states handled by this handler
	handled_states.insert(CameraState::DISCONNECTED_STATE);

	std::stringstream err_stream;
	err_stream << "Disconneced" << std::endl;
	this->emit_status_message(err_stream.str());

	retval = this->stop_camera_acquisition();
	if (retval != 0)
	{
		err_stream << "disconnected_handler: Error stopping camera acquisition, returned " << this->get_error_string(retval) << std::endl;
		this->emit_status_message(err_stream.str());
	}
	retval = this->close_camera();
	if (retval != 0)
	{
		err_stream << "disconnected_handler: Error closing camera, returned " << this->get_error_string(retval) << std::endl;
		this->emit_status_message(err_stream.str());
	}

	while (this->stop_state_flag == false)
	{
		// Check if there is a command and react to appropriate commands in a switch statement:
		cmd_data = this->check_commands();
		switch(cmd_data.command)
		{
			case CameraCommand::INIT_CMD:
				new_state = CameraState::INIT_STATE;
				break;
			case CameraCommand::START_CAPTURE_CMD:
				new_state = CameraState::RUNNING_STATE;
				break;
			case CameraCommand::CONNECT_CMD:
				new_state = CameraState::IDLE_STATE;
				break;

			case CameraCommand::NO_CMD:
			default:
				break;
		};

		// If the state is not among those handled, break out of the while loop:
		if (handled_states.find(new_state) == handled_states.end()) {
			break;
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		};
	};
	return new_state;
}; // JaiGenicamCameraControl::disconnected_handler



/** Handler function for the IDLE_STATE. We are connected to the camera, but not aquiring images. It waits in a loop checking commands periodically. 
*/
CameraState JaiGenicamCameraControl::idle_handler(CameraState prev_state)
{
	// Lock that can be re-locked as needed for queue access:
	std::unique_lock<std::mutex> lock(this->queue_mutex, std::defer_lock);
	CameraCommandData cmd_data;		// Variable for storing incoming commands
	GenicamGenericNode generic_node; // Variable for setting nodes
	int retval;
	CameraState new_state = CameraState::IDLE_STATE;	// Variable for storing shift in state
	std::set<CameraState> handled_states;				// Set of states handled by this handler
	handled_states.insert(CameraState::IDLE_STATE);

	this->emit_status_message("Idle");

	std::stringstream err_stream;

	std::cout << "idle_handler: stop acq" << std::endl;
	// Stop camera acquisition
	retval = this->stop_camera_acquisition();
	if (retval != 0)
	{
		err_stream << "idle_handler: Error stopping camera acquisition, returned " << this->get_error_string(retval) << std::endl;
		std::cout << err_stream.str() << std::endl;
		this->emit_status_message(err_stream.str());
		new_state = CameraState::FAULT_STATE;
	}

	// Variables to do periodic camera check
	double check_interval = 0.5;
	double current_time = (std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
	double last_time = current_time;

	while (this->stop_state_flag == false)
	{
		// Check if there is a command and react to appropriate commands in a switch statement:
		cmd_data = this->check_commands();
		switch(cmd_data.command)
		{
			case CameraCommand::INIT_CMD:
				new_state = CameraState::INIT_STATE;
				break;

			case CameraCommand::GET_NODE_CMD:
//				std::cout << " Idle_handler GET_NODE_CMD " << cmd_data.name_string << std::endl;
				retval = this->generate_genericnode_from_name(cmd_data.name_string, generic_node);
				if (retval != 0)
				{
					this->set_error_state(CameraState::IDLE_STATE);
					new_state = CameraState::FAULT_STATE;
					std::stringstream err_stream;
					err_stream << "idle_handler: Error generate_generic_node_from_name " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
//					this->emit_status_message(err_stream.str());
				}
				else
				{
					lock.lock();
					this->node_map[cmd_data.name_string] = generic_node;
					lock.unlock();
//					std::cout << " Idle_handler GET_NODE_CMD... node " << cmd_data.name_string << std::endl;
				}
				break;

			case CameraCommand::SET_NODE_CMD:	
				std::cout << " Idle_handler SET_NODE_CMD " << cmd_data.name_string << std::endl;
				retval = this->get_node(cmd_data.name_string, generic_node);
				if (retval != 0)
				{
					err_stream << cmd_data.name_string << " not found in node map";
					this->set_error_data("idle_handler", "SET_NODE_CMD", err_stream.str(), -1, CameraState::IDLE_STATE, false);
					new_state = CameraState::FAULT_STATE;
					break;
				}
				
				generic_node.name = cmd_data.name_string;
				generic_node.value_d = cmd_data.data_float;
				generic_node.value_i = cmd_data.data_int;
				generic_node.value_s = cmd_data.data_string;
				
				retval = this->set_node_to_camera(generic_node);
				if (retval != 0)
				{
					this->set_error_state(CameraState::IDLE_STATE);
					new_state = CameraState::FAULT_STATE;
					std::stringstream err_stream;
					err_stream << "idle_handler: Error set_node_to_camera " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
					std::cout << err_stream.str();
//					this->emit_status_message(err_stream.str());
				};
				break;

			case CameraCommand::START_CAPTURE_CMD:
				new_state = CameraState::RUNNING_STATE;
				break;
			case CameraCommand::DISCONNECT_CMD:
				new_state = CameraState::DISCONNECTED_STATE;
				break;

			case CameraCommand::NO_CMD:
			default:
				break;
		};

		// If the state is not among those handled, break out of the while loop:
		if (handled_states.find(new_state) == handled_states.end()) {
			break;
		}
		else {
			current_time = (std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
			if (current_time - last_time > check_interval)
			{
				if (this->exposuretime_node_name.empty() == false)
				{
					last_time = current_time;
					std::string node_name = this->exposuretime_node_name;
					retval = this->get_node(node_name, generic_node);
					if (retval != 0)
					{
						err_stream << node_name << " not found in node map";
						this->set_error_data("idle_handler", "SET_NODE_CMD", err_stream.str(), -1, CameraState::IDLE_STATE, false);
						new_state = CameraState::FAULT_STATE;
						break;
					}
					
					retval = this->set_node_to_camera(generic_node);
					if (retval != 0)
					{
						this->set_error_state(CameraState::IDLE_STATE);
						new_state = CameraState::FAULT_STATE;
						std::stringstream err_stream;
						err_stream << "idle_handler: Error set_node_to_camera " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
//						this->emit_status_message(err_stream.str());
					}
				}
			};
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		};
	};
	return new_state;
}; // JaiGenicamCameraControl::idle_handler


/** Handler function for the RUNNING_STATE. Start aquiring images. It waits in a loop checking commands periodically. 
*/
CameraState JaiGenicamCameraControl::running_handler(CameraState prev_state)
{
	// Lock that can be re-locked as needed for queue access:
	std::unique_lock<std::mutex> lock(this->queue_mutex, std::defer_lock);
	CameraCommandData cmd_data;		// Variable for storing incoming commands
	GenicamGenericNode generic_node; // Variable for setting nodes
	int retval;
	CameraState new_state = CameraState::RUNNING_STATE;	// Variable for storing shift in state
	std::set<CameraState> handled_states;				// Set of states handled by this handler
	handled_states.insert(CameraState::RUNNING_STATE);

	std::stringstream err_stream;
	std::stringstream status_stream;

	// Variables to do periodic camera check
	double check_interval = 0.5;
	double current_time = (std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
	double last_time = current_time;


	retval = this->start_camera_acquisition();
	if (retval != 0)
	{
		this->set_error_state(CameraState::RUNNING_STATE);
		err_stream << "running_handler: Error starting camera acquisition, returned " << this->get_error_string(retval) << std::endl;
//		this->emit_status_message(err_stream.str());
		new_state = CameraState::FAULT_STATE;
	}

	status_stream << "Camera running" << std::endl;
	this->emit_status_message(status_stream.str());

	while (this->stop_state_flag == false)
	{
		// Check if there is a command and react to appropriate commands in a switch statement:
		cmd_data = this->check_commands();
		switch(cmd_data.command)
		{
			case CameraCommand::INIT_CMD:
				new_state = CameraState::INIT_STATE;
				break;

			case CameraCommand::GET_NODE_CMD:
				retval = this->generate_genericnode_from_name(cmd_data.name_string, generic_node);
				if (retval != 0)
				{
					this->set_error_state(CameraState::RUNNING_STATE);
					new_state = CameraState::FAULT_STATE;
					std::stringstream err_stream;
					err_stream << "running_handler: Error generate_generic_node_from_name " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
//					this->emit_status_message(err_stream.str());
				}
				else
				{
					lock.lock();
					this->node_map[cmd_data.name_string] = generic_node;
					lock.unlock();
				}
				break;

			case CameraCommand::SET_NODE_CMD:		
				retval = this->get_node(cmd_data.name_string, generic_node);
				if (retval != 0)
				{
					err_stream << cmd_data.name_string << " not found in node map";
					this->set_error_data("running_handler", "SET_NODE_CMD", err_stream.str(), -1, CameraState::RUNNING_STATE, false);
					new_state = CameraState::FAULT_STATE;
					break;
				}
				generic_node.name = cmd_data.name_string;
				generic_node.value_d = cmd_data.data_float;
				generic_node.value_i = cmd_data.data_int;
				generic_node.value_s = cmd_data.data_string;
				
				std::cout << " Running_handler SET_NODE_CMD " << cmd_data.name_string << std::endl;
				retval = this->set_node_to_camera(generic_node);
				if (retval != 0)
				{
					this->set_error_state(CameraState::RUNNING_STATE);
					new_state = CameraState::FAULT_STATE;
					std::stringstream err_stream;
					err_stream << "running_handler: Error set_node_to_camera " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
//					this->emit_status_message(err_stream.str());
				};
				break;
			case CameraCommand::STOP_CAPTURE_CMD:
			case CameraCommand::CONNECT_CMD:
				new_state = CameraState::IDLE_STATE;
				break;
			case CameraCommand::DISCONNECT_CMD:
				new_state = CameraState::DISCONNECTED_STATE;
				break;

			case CameraCommand::NO_CMD:
			default:
				break;
		};

		// If the state is not among those handled, break out of the while loop:
		if (handled_states.find(new_state) == handled_states.end()) {
			break;
		}
		else 
		{
			if (current_time - last_time > check_interval)
			{
				if (this->exposuretime_node_name.empty() == false)
				{
					std::cout << "running_handler: Setting exposuretime in order to check the camera" << std::endl;
					last_time = current_time;
					std::string node_name = "ExposureTime";
					retval = this->get_node(node_name, generic_node);
					if (retval != 0)
					{
						err_stream << cmd_data.name_string << " not found in node map";
						this->set_error_data("running_handler", "ExposureTime", err_stream.str(), -1, CameraState::RUNNING_STATE, false);
						new_state = CameraState::FAULT_STATE;
						break;
					}
					retval = this->set_node_to_camera(generic_node);
					if (retval != 0)
					{
						this->set_error_state(CameraState::RUNNING_STATE);
						new_state = CameraState::FAULT_STATE;
						std::stringstream err_stream;
						err_stream << "running_handler: Error generate_generic_node_from_name " << cmd_data.name_string << ", returned " << this->get_error_string(retval) << std::endl;
//						this->emit_status_message(err_stream.str());
					}
				}
			};
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		};
	};
//	this->stop_camera_acquisition();
	return new_state;
}; // JaiGenicamCameraControl::running_handler


/** Handler function for the FAULT_STATE. An error was detected. Try to diagnose and exit to the previous state. 
*/
CameraState JaiGenicamCameraControl::fault_handler(CameraState prev_state)
{
	std::stringstream	status_stream;
	std::string			status_string;
	int retval;
	int retries = 0;
	int n_retries = 10;
	CameraState new_state = CameraState::UNKNOWN_STATE;

	status_stream << "Fault detected" << std::endl;
	GenicamErrorStruct error_data = this->get_error_data();
	status_stream << "=====================================================" << std::endl;
	status_stream << "From " << error_data.calling_function << " in " << error_data.camera_function << " returned: " << std::endl;
	status_stream << error_data.error_message << std::endl << std::endl;

	std::cout << status_stream.str();

	this->emit_status_message(status_stream.str());
	if (error_data.retval != 0)
	{
		while ((retries < n_retries) && (this->stop_state_flag == false))
		{			
			if (error_data.calling_state == CameraState::NO_STATE)
			{
				error_data.state_string = "FAULT";
				error_data.calling_state = CameraState::FAULT_STATE;
			}
			this->error_signal.emit(error_data);

			// Find specific errors:
			if ((error_data.retval == J_STATUS_CODES::J_ST_ACCESS_DENIED) && (error_data.camera_function == "J_Camera_Open"))
			{
				std::cout << "Camera access error!" << std::endl;
				status_string = status_stream.str() + "Camera could be open in another application. Check and close camera.";
				this->emit_status_message(status_string);

				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				retval = this->open_camera();
				if (retval == 0)
				{
					new_state = prev_state;
					break;
				};
			}			
			else if ((error_data.camera_function == "J_Factory_GetCameraInfo") && (error_data.error_message.find("serial") != std::string::npos))
			{
				std::cout << "Camera not found on bus!" << std::endl;
				status_string = status_stream.str() + "Camera not found. Check if the camera is connected.";
				this->emit_status_message(status_string);

				retval = this->open_factory();
				if (retval != 0)
				{					
					new_state = CameraState::FAULT_STATE;
					break;
				}
				int8_t          camera_id_s[J_CAMERA_ID_SIZE];
				retval = this->find_camera(this->camera_serial, camera_id_s);
				if (retval == 0)
				{
					std::cout << "Finally found camera!" << std::endl;
					new_state = prev_state;
					break;
				}
			}
			else if ((error_data.camera_function == "SET_NODE_CMD") && (error_data.error_message.find("not found") != std::string::npos))
			{
				std::cout << "SET_NODE_CMD: Node name not found." << std::endl;
				std::string node_name = error_data.error_message.substr(0, error_data.error_message.find(" "));
				status_string = "Node named '" + node_name + "' not found. Check if there is such a node in the camera. If not, rename the device server property and re-initialize device server.";
				this->set_sticky_status_message(status_string, true);
				this->emit_status_message(status_stream.str());
				new_state = prev_state;

				// Normally each SET_NODE_CMD is followed by a GET_NODE_CMD.
				// Then we will get an additional unnecessary error.
				// Remove this command from the queue if it exist.
				CameraCommandData cmd = this->check_commands();
				if (cmd.command != CameraCommand::GET_NODE_CMD)
				{
					this->send_command(cmd);
				}
				break;
			}
			else if ((error_data.camera_function == "J_Camera_GetNodeByName") && (error_data.retval == J_STATUS_CODES::J_ST_INVALID_PARAMETER))
			{
				std::cout << "J_Camera_GetNodeByName: Node name not found." << std::endl;
				std::string node_name = error_data.error_message.substr(0, error_data.error_message.find(" "));
				status_string = "Node named '" + node_name + "' not found. Check if there is such a node in the camera. If not, rename the device server property and re-initialize device server.";
				this->set_sticky_status_message(status_string, true);
				this->emit_status_message(status_stream.str());
				new_state = prev_state;
				break;
			}
			else if ((error_data.retval == J_STATUS_CODES::J_ST_INVALID_PARAMETER))
			{
				std::cout << "Invalid parameter error!" << std::endl;
				status_string = status_stream.str() + "Parameter could be wrong for the node.";
				this->emit_status_message(status_string);

				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			else if ((error_data.retval == J_STATUS_CODES::J_ST_GC_ERROR))
			{
				std::cout << "GenICam error!" << std::endl;
				status_string = status_stream.str();
				this->emit_status_message(status_string);

				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			else
			{
				// Generic "fault handling": wait 2 seconds and restart from UNKNOWN state.
				new_state = CameraState::UNKNOWN_STATE;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				break;
			};
						
			retries += 1;
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	};
	

	this->clear_error_data();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	return new_state;
}; // JaiGenicamCameraControl::fault_handler