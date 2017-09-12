#include "JaiGenicamCameraControl.h"
#include <set>
#include <stack>

using namespace JaiGenicamCameraControl_ns;

JaiGenicamCameraControl::~JaiGenicamCameraControl(void)
{
	std::cout << "======= Entrering JaiCameraControl destructor =======" << std::endl;
	std::cout << "Current state: " << this->get_state() << std::endl;
	this->stop_state_flag = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	std::cout << "Current state: " << this->get_state() << std::endl;
	this->state_handler_thread.join();
	std::cout << "JaiCameraControl destructor: thread joined " << std::endl;
	this->close_factory();
}


/** Return the current executing state
*/
CameraState JaiGenicamCameraControl::get_state()
{

	std::lock_guard<std::mutex> lock(this->queue_mutex);
	CameraState state = this->state;
	return state;
};


/** Return cached node data
*/
int JaiGenicamCameraControl::get_node(std::string node_name, GenicamGenericNode &node)
{
	std::lock_guard<std::mutex> lock(this->node_mutex);
	try
	{
		node = this->node_map.at(node_name);
	}
	catch (const std::out_of_range& oor)
	{
		node.name = node_name;
		node.valid = false;
		return J_ST_INVALID_ID;
	};
	if (node.valid == true)
	{
		return 0;
	}
	else
	{
		std::cout << "get_node: Found invalid node: " << node_name << std::endl;
		return J_ST_INVALID_PARAMETER;
	}
	return 0;
};

int JaiGenicamCameraControl::get_node_value(std::string name, double &value)
{
	GenicamGenericNode node;
	std::lock_guard<std::mutex> lock(this->node_mutex);
	try
	{
		node = this->node_map.at(name);
	}
	catch (const std::out_of_range& oor)
	{
//		std::cout << "JaiGenicamCameraControl::get_node_value(std::string name, double &value): Out of range error in map. " << name << std::endl;

		node.name = name;
		node.valid = false;
		value = 0;
		return -1;
	};
	switch (node.type)
	{
	case J_IFloat:
		value = node.value_d;
		break;
	case J_IInteger:
	case J_IBoolean:
		value = node.value_i;
		break;
	default:
		node.name = name;
		node.valid = false;
		value = 0;
		return -2;
	};
	if (node.valid == true)
	{
		return 0;
	}
	else
	{
		std::cout << "get_node_value: Found invalid node: " << name << std::endl;
		return -3;
	}
	return 0;
};

int JaiGenicamCameraControl::get_node_value(std::string name, int64_t &value)
{
	GenicamGenericNode node;
	std::lock_guard<std::mutex> lock(this->node_mutex);
	try
	{
		node = this->node_map.at(name);
	}
	catch (const std::out_of_range& oor)
	{
//		std::cout << "JaiGenicamCameraControl::get_node_value(std::string name, int64_t &value): Out of range error in map. " << name << std::endl;
		node.name = name;
		node.valid = false;
		value = 0;
		return -1;
	};
	value = node.value_i;
	if (node.valid == true)
	{
		return 0;
	}
	else
	{
		std::cout << "get_node_value: Found invalid node: " << name << std::endl;
		return -1;
	}
	return 0;
};

/** Add a command on the queue to set a node to the value in the GenicamGenericNode.
Then add another command to read the node back to the cached list.
*/
int JaiGenicamCameraControl::set_node(GenicamGenericNode node)
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::SET_NODE_CMD;
	cmd_data.name_string = node.name;
	cmd_data.data_float = node.value_d;
	cmd_data.data_int = node.value_i;
	cmd_data.data_string = node.value_s;
	this->send_command(cmd_data);
	cmd_data.command = CameraCommand::GET_NODE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::set_node_value(std::string name, double value)
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::SET_NODE_CMD;
	cmd_data.name_string = name;
	cmd_data.data_float = value;
	cmd_data.data_int = int(value);
	cmd_data.data_string = std::to_string(value);
	this->send_command(cmd_data);
	cmd_data.command = CameraCommand::GET_NODE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::set_node_value(std::string name, int64_t value)
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::SET_NODE_CMD;
	cmd_data.name_string = name;
	cmd_data.data_float = value;
	cmd_data.data_int = value;	
	cmd_data.data_string = std::to_string(value);
	this->send_command(cmd_data);
	cmd_data.command = CameraCommand::GET_NODE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::set_node_value(std::string name, std::string value)
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::SET_NODE_CMD;
	cmd_data.name_string = name;
//	cmd_data.data_float = std::stod(value);
//	cmd_data.data_int = std::stoi(value);
	cmd_data.data_string = value;
	GenicamGenericNode generic_node;
	{
		std::lock_guard<std::mutex> lock(this->queue_mutex);
		try
		{
			generic_node = this->node_map.at(name);
		}
		catch (const std::out_of_range& oor)
		{
			std::cout << "JaiGenicamCameraControl::set_node_value: Enum entry " << value << " for node " << name << " not found in map." << std::endl;
			return -1;
		};
	};
	// If it is a enumeration node that is being set with a string, retrieve the enum value corresponding to that string:
	if (generic_node.type == J_IEnumeration)
	{
		try
		{
			cmd_data.data_int = generic_node.enum_entry_map.at(value);
		}
		catch (const std::out_of_range& oor)
		{
			std::cout << "JaiGenicamCameraControl::set_node_value: Enum entry " << value << " for node " << name << " not found." << std::endl;
			return -1;
		};
	};
	this->send_command(cmd_data);
	cmd_data.command = CameraCommand::GET_NODE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::get_node_info(std::string name, GenicamGenericNode &generic_node)
{
	GenicamGenericNode node;
	std::lock_guard<std::mutex> lock(this->node_mutex);
	try
	{
		node = this->node_map.at(name);
//		std::cout << node.description << std::endl;
	}
	catch (const std::out_of_range& oor)
	{
		node.name = name;
		node.valid = false;
		return -1;
	};
	generic_node = node;
	return 0;
};

int JaiGenicamCameraControl::get_node_type(std::string name, std::string &type)
{
	GenicamGenericNode node;
	std::lock_guard<std::mutex> lock(this->node_mutex);
	try
	{
		node = this->node_map.at(name);
	}
	catch (const std::out_of_range& oor)
	{
		node.name = name;
		node.valid = false;
		type = "UNKNOWN";
		return -1;
	};
	type = node.type;
	return 0;
};


/** Re-read info from camera for a specific node and update the information in the node map.
This can be used e.g. if limits for a node have changed due to writing another node (such as 
width - offset_x).
*/
int JaiGenicamCameraControl::update_nodeinfo(std::string node_name)
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::UPDATE_NODE_CMD;
	cmd_data.name_string = node_name;
	this->send_command(cmd_data);
	return 0;
} // JaiGenicamCameraControl::update_nodemap_node


/** Add a command on the queue to set a node to the value in the GenicamGenericNode.
Then add another command to read the node back to the cached list.
*/
int JaiGenicamCameraControl::disconnect()
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::DISCONNECT_CMD;
	this->send_command(cmd_data);
	return 0;
};

/** Add a command on the queue to set a node to the value in the GenicamGenericNode.
Then add another command to read the node back to the cached list.
*/
int JaiGenicamCameraControl::connect()
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::CONNECT_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::start_capture()
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::START_CAPTURE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::stop_capture()
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::STOP_CAPTURE_CMD;
	this->send_command(cmd_data);
	return 0;
};

int JaiGenicamCameraControl::reset()
{
	CameraCommandData cmd_data;
	cmd_data.command = CameraCommand::RESET_CAMERA_CMD;
	this->send_command(cmd_data);
	return 0;
};

/** Put a new command on the command_queue.
*/
void JaiGenicamCameraControl::send_command(CameraCommandData command_data)
{
	std::lock_guard<std::mutex> lock(this->queue_mutex);
	this->command_queue.push(command_data);
};

/** Check the command queue for commands to execute. The queue is accessed protected by queue_mutex lock.
If  the queue is empty a NO_CMD command is returned.
*/
CameraCommandData JaiGenicamCameraControl::check_commands()
{
	CameraCommandData cmd;
	std::lock_guard<std::mutex> lock(this->queue_mutex);
	if (this->command_queue.empty() == false) {
		cmd = this->command_queue.front();
		this->command_queue.pop();
		
	}
	else {
		// If the command queue was empty return a no_cmd:
		cmd.command = CameraCommand::NO_CMD;
	};
	return cmd;
};

std::string JaiGenicamCameraControl::get_error_string(J_STATUS_TYPE error)
{
	std::stringstream error_msg;
	
	switch(error)
	{
	case J_ST_INVALID_BUFFER_SIZE:	error_msg << "Invalid buffer size. ";				break;
	case J_ST_INVALID_HANDLE:		error_msg << "Invalid handle. ";		            break;
	case J_ST_INVALID_ID:			error_msg << "Invalid ID. ";			            break;
	case J_ST_ACCESS_DENIED:		error_msg << "Access denied. ";		                break;
	case J_ST_NO_DATA:				error_msg << "No data. ";							break;
	case J_ST_ERROR:				error_msg << "Generic error. ";		                break;
	case J_ST_INVALID_PARAMETER:	error_msg << "Invalid parameter. ";	                break;
	case J_ST_TIMEOUT:				error_msg << "Timeout. ";							break;
	case J_ST_INVALID_FILENAME:		error_msg << "Invalid file name. ";	                break;
	case J_ST_INVALID_ADDRESS:		error_msg << "Invalid address. ";					break;
	case J_ST_FILE_IO:				error_msg << "File IO error. ";		                break;
	case J_ST_GC_ERROR:				error_msg << "GenICam error. ";		                break;
	case J_ST_VALIDATION_ERROR:		error_msg << "Settings File Validation Error. ";	break;
	case J_ST_VALIDATION_WARNING:	error_msg << "Settings File Validation Warning. ";	break;
	}
	error_msg << "(code " << std::to_string(error) << ")";
	return error_msg.str();
}


/** Lock the error mutex and fill in the internal error_data struct. This info is used by the fault handler.
*/
int JaiGenicamCameraControl::set_error_data(std::string calling_function, std::string camera_function, std::string error_message, int retval, CameraState calling_state = CameraState::NO_STATE, bool emit_signal=false)
{
	GenicamErrorStruct error_data;
	error_data.calling_state = calling_state;
	error_data.calling_function = calling_function;
	error_data.camera_function = camera_function;
	error_data.error_message = error_message;
	error_data.retval = retval;
	switch (calling_state)
	{
	case CameraState::DISCONNECTED_STATE:
		error_data.state_string = "DISCONNECTED";
		break;
	case CameraState::FAULT_STATE:
		error_data.state_string = "FAULT";
		break;
	case CameraState::IDLE_STATE:
		error_data.state_string = "IDLE";
		break;
	case CameraState::INIT_STATE:
		error_data.state_string = "INIT";
		break;
	case CameraState::RUNNING_STATE:
		error_data.state_string = "RUNNING";
		break;
	case CameraState::UNKNOWN_STATE:
		error_data.state_string = "UNKNOWN";
		break;

	default:
		error_data.state_string = "--";
		break;
	}
	{
		std::lock_guard<std::mutex> lock(this->error_mutex);
		this->error_data = error_data;		
	}
	if (emit_signal == true)
	{
		this->error_signal.emit(error_data);
	}
	return 0;
} // JaiGenicamCameraControl::set_error_data


/** Set calling state part of the internal error_data variable.
This is needed as a subfunction is not aware of the current state.
*/
int JaiGenicamCameraControl::set_error_state(CameraState state)
{
	{
		std::lock_guard<std::mutex> lock(this->error_mutex);
		this->error_data.calling_state = state;		
		switch (state)
		{
		case CameraState::DISCONNECTED_STATE:
			this->error_data.state_string = "DISCONNECTED";
			break;
		case CameraState::FAULT_STATE:
			this->error_data.state_string = "FAULT";
			break;
		case CameraState::IDLE_STATE:
			this->error_data.state_string = "IDLE";
			break;
		case CameraState::INIT_STATE:
			this->error_data.state_string = "INIT";
			break;
		case CameraState::RUNNING_STATE:
			this->error_data.state_string = "RUNNING";
			break;
		case CameraState::UNKNOWN_STATE:
			this->error_data.state_string = "UNKNOWN";
			break;

		default:
			this->error_data.state_string = "--";
			break;
		}
	}
	return 0;
} // JaiGenicamCameraControl::set_error_state


/** Lock the error mutex and read out the error_data struct.
*/
GenicamErrorStruct JaiGenicamCameraControl::get_error_data()
{
	GenicamErrorStruct error_data;
	{
		std::lock_guard<std::mutex> lock(this->error_mutex);
		error_data = this->error_data;		
	}
	
	return error_data;
}; // JaiGenicamCameraControl::get_error_data


/** Lock the error mutex and clear the internal error_data struct.
*/
int JaiGenicamCameraControl::clear_error_data()
{
	GenicamErrorStruct error_data;
	error_data.calling_state = CameraState::NO_STATE;
	error_data.calling_function.clear();
	error_data.camera_function.clear();
	error_data.retval = 0;
	{
		std::lock_guard<std::mutex> lock(this->error_mutex);
		this->error_data = error_data;		
	}
	return 0;
};


/** Set the sticky status message that is appended to every message signal sent with emit_status_message
*/
int JaiGenicamCameraControl::set_sticky_status_message(std::string sticky_status_msg, bool append=true)
{
	{
		std::stringstream s;
		std::lock_guard<std::mutex> lock(this->error_mutex);
		if (append == true)
		{
			s << this->sticky_status_message << std::endl << sticky_status_msg;
			this->sticky_status_message = s.str();
		}
		else
		{
			this->sticky_status_message = sticky_status_msg;
		}
	}
	return 0;
} // JaiGenicamCameraControl::set_sticky_status_message


/** Emit the status_message_signal, including the sticky status message
*/
int JaiGenicamCameraControl::emit_status_message(std::string status_msg)
{
	std::stringstream status_stream;
	status_stream << status_msg << std::endl << std::endl;
	{
		std::lock_guard<std::mutex> lock(this->error_mutex);
		status_stream << sticky_status_message;
	}
	this->status_message_signal.emit(status_stream.str());
	return 0;
} // JaiGenicamCameraControl::emit_status_message

/** -------------------------------------------------------------------------
*
*    Camera access specific methods
*
*  --------------------------------------------------------------------------*/


/** ===================================================================
Callback for new image ready from the JaiGenicamSDK. The new image is copied into the image_buffer.

Return 0 if successful, otherwise return the genicam error code.
*/
void __stdcall JaiGenicamCameraControl::capture_stream_callback(J_tIMAGE_INFO * aq_imageinfo_p)
{
	J_STATUS_TYPE   retval;
	std::stringstream err_msg;
	//std::cout << "capture_stream_callback: New image acquired" << std::endl;

	std::unique_lock<std::mutex> lock(this->camera_mutex);
	double frame_time;
	int new_framecounter;
	frame_time = (std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
	// The state of the buffer is confirmed.

	if(this->image_buffer.pImageBuffer != NULL)
	{
		// When the buffer already exists, and the size is different:
		if((this->image_buffer.iSizeX != aq_imageinfo_p->iSizeX) || (this->image_buffer.iSizeY != aq_imageinfo_p->iSizeY) || (this->image_buffer.iPixelType != aq_imageinfo_p->iPixelType))
		{
			// Abandons the buffer.
			retval = J_Image_Free(&this->image_buffer);
			if (retval != J_ST_SUCCESS)
			{
				err_msg << "Could not free buffer!" << this->get_error_string(retval);
				std::cout << err_msg.str() << std::endl;
				this->set_error_data("capture_stream_callback", "J_Image_Free", err_msg.str(), retval);
			};
			this->image_buffer.pImageBuffer = NULL;
		}
	}

	// Allocates it when there is no buffer.
	if(this->image_buffer.pImageBuffer == NULL)
	{
		retval = J_Image_Malloc(aq_imageinfo_p, &this->image_buffer);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not allocate buffer!" << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("capture_stream_callback", "J_Image_Malloc", err_msg.str(), retval);
		};
	};

	// The image making is done for the picture processing.
	retval = J_Image_FromRawToImage(aq_imageinfo_p, &this->image_buffer);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not convert to RAW!" << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("capture_stream_callback", "J_Image_FromRawToImage", err_msg.str(), retval);
	};

	// Update frame counter
	new_framecounter = this->frame_counter + 1;
	this->frame_counter = new_framecounter;

	// Update fps_vector
	if (fps_vector.size() > 10) 
	{
		fps_vector.erase(fps_vector.begin());
	}
	fps_vector.push_back(frame_time);
	lock.unlock();
	this->image_ready_signal.emit(new_framecounter);
    
}; // JaiGenicamCameraControl::capture_stream_callback


/** Update node map data from camera
*/
int JaiGenicamCameraControl::get_node_from_camera(std::string name, GenicamGenericNode &generic_node)
{
	int retval;
	

	return 0;
};	// JaiGenicamCameraControl::get_node_from_camera


/** Return image info structure from the internal image_buffer.
camera_mutex is locked during buffer access.
*/
int JaiGenicamCameraControl::get_image_info(J_tIMAGE_INFO &aq_image_info)
{
	int retval = -1;
	std::unique_lock<std::mutex> lock(this->camera_mutex);		
	if (this->image_buffer.pImageBuffer != NULL)
	{
		aq_image_info.iImageSize = this->image_buffer.iImageSize;
		aq_image_info.iMissingPackets = this->image_buffer.iMissingPackets;
		aq_image_info.iOffsetX = this->image_buffer.iOffsetX;
		aq_image_info.iOffsetY = this->image_buffer.iOffsetY;
		aq_image_info.iPixelType = this->image_buffer.iPixelType;
		aq_image_info.iQueuedBuffers = this->image_buffer.iQueuedBuffers;
		aq_image_info.iSizeX = this->image_buffer.iSizeX;
		aq_image_info.iSizeY = this->image_buffer.iSizeY;
		aq_image_info.iTimeStamp = this->image_buffer.iTimeStamp;
		aq_image_info.iAnnouncedBuffers = this->image_buffer.iAnnouncedBuffers;
		aq_image_info.iAwaitDelivery = this->image_buffer.iAwaitDelivery;
		aq_image_info.pImageBuffer = NULL;
		retval = 0;
//				std::cout << "Image size: " << aq_image_info.iImageSize << std::endl;
	}
	else
	{
		retval = -1;
	};

	lock.unlock();
	return retval;

}; // JaiGenicamCameraControl::get_image_info


/** Return the internal framecounter.
camera_mutex is locked during framecounter access-
*/
int JaiGenicamCameraControl::get_framecounter(int64_t &value)
{
	std::cout << "get_framecounter: " << this->frame_counter << std::endl;
	std::lock_guard<std::mutex> lock(this->camera_mutex);
	value = this->frame_counter;
	std::cout << value << std::endl;
	return 0;
} // JaiGenicamCameraControl::get_framecounter


/** Calculate framerate from fps_vector timestamps. Return the calculated framerate.
*/
int JaiGenicamCameraControl::get_framerate(double &value)
{
	double current_time = (std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
	double fps_mean = 0;
	double dt_last = 0;
	std::unique_lock<std::mutex> lock(this->camera_mutex);
			
		if (this->fps_vector.size() > 1) {
			double dt_mean = (this->fps_vector.back() - this->fps_vector.front()) / (this->fps_vector.size() - 1);

			dt_last = current_time - this->fps_vector.back();
			if (dt_last > 2*dt_mean)
			{
				// We have a substantial delay without getting a new frame, adjust dt
				dt_mean = (current_time - this->fps_vector.front()) / (this->fps_vector.size());
			}
			fps_mean = 1 / dt_mean;		
		}
		value = fps_mean;
		lock.unlock();
	return 0;		
}; // JaiGenicamCameraControl::get_framerate


/** Copy the image in the image_buffer to *image_p. Image height and width is returned in width and height.
camera_mutex is locked during the copy operation.

*/
int JaiGenicamCameraControl::get_image(uint32_t &width, uint32_t &height, uint16_t* image_p)
{
	int retval = -1;
//		std::cout << "Entering JaiGenicamState::get_image" << std::endl;
	std::unique_lock<std::mutex> lock(this->camera_mutex);		
	if (this->image_buffer.pImageBuffer != NULL)
	{			
		width = this->image_buffer.iSizeX;
		height = this->image_buffer.iSizeY;
		// Check if there are 2 bytes per pixel or 1 byte per pixel:
		if (this->image_buffer.iImageSize / (this->image_buffer.iSizeX * this->image_buffer.iSizeY) < 2)
		{
			// 1 Bpp so do a copy
			std::copy(this->image_buffer.pImageBuffer, 
			this->image_buffer.pImageBuffer+(width)*(height)-1,
			image_p);					
		}
		else
		{
			// 1 Bpp so first reinterpret_cast to uint16_t before copy
			uint16_t* tmp_image_p = reinterpret_cast<uint16_t*>(this->image_buffer.pImageBuffer);
			std::copy(tmp_image_p, 
			tmp_image_p+(width)*(height)-1,
			image_p);	
		}


		retval = 0;
	};
	lock.unlock();
	return retval;
}; // JaiGenicamCameraControl::get_image


/** Check if the supplied string is formatted as a valid ip address
*/
bool JaiGenicamCameraControl::is_ipaddress(std::string ip_string)
{
	std::vector<std::string> quads;
	std::string current_string = ip_string;
	size_t pos = 0;
	size_t next_pos = current_string.find(".", pos);
	while (pos != current_string.npos)
	{
		quads.push_back(current_string.substr(pos, next_pos - pos));
		if (next_pos != std::string::npos)
		{
			pos = next_pos + 1;
			next_pos = current_string.find(".", pos);
		}
		else
		{
			pos = next_pos;
		}
	};

    if (quads.size() != 4) return false;

    for (int i=0; i < 4; i++)
	{
		std::string quad = quads[i];
		for (int j=0; j < quad.length(); j++)
		{
			if (!isdigit(quad[j])) return false;
		}
		int quad_nbr = std::atoi(quads[i].c_str());
		if ((quad_nbr < 0) || (quad_nbr > 255)) return false;
	}

    return true;
}; // JaiGenicamCameraControl::is_ipaddress