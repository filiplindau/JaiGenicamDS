#include "JaiGenicamCameraControl.h"
#include <set>
#include <stack>

using namespace JaiGenicamCameraControl_ns;



/** Open the jai genicam factory. If the was already opened, close it first.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::open_factory()
{
	std::cout << "JaiGenicamCameraControl::open_factory" << std::endl;
	J_STATUS_TYPE   retval;

	std::stringstream err_msg;

	std::unique_lock<std::mutex> lock(this->camera_mutex, std::defer_lock);

	lock.lock();
	// Close factory if already open
	if (this->factory_handle != NULL)
	{
		lock.unlock();
		retval = this->close_factory();
		if (retval != 0)
		{
			return retval;
		}
		lock.lock();
	};
	if (this->factory_handle == NULL)
	{
		retval = J_Factory_Open((int8_t*)"" , &this->factory_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not open factory!" << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("open_factory", "J_Factory_Open", err_msg.str(), retval, CameraState::UNKNOWN_STATE, false);
			return retval;
		}
		else
		{
//			std::cout << "Factory open." << std::endl;
		};
	};
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::open_factory


/** Close the jai genicam factory. 

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::close_factory()
{
	std::cout << "JaiGenicamCameraControl::close_factory" << std::endl;
	J_STATUS_TYPE   retval;
	std::stringstream err_msg;
	std::unique_lock<std::mutex> lock(this->camera_mutex, std::defer_lock);

	lock.lock();
	// Close camera if already open
	if (this->camera_handle != NULL)
	{
		lock.unlock();
		retval = this->close_camera();
		if (retval != 0)
		{
			return retval;
		}
		lock.lock();
	};
	if (this->factory_handle != NULL)
	{
		retval = J_Factory_Close(this->factory_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not close factory!" << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("close_factory", "J_Factory_Close", err_msg.str(), retval, CameraState::UNKNOWN_STATE, false);
			return retval;
		}
		else
		{
//			std::cout << "Factory closed." << std::endl;
			this->factory_handle = NULL;
		}
	}
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::close_factory



/** ===================================================================
Open a connection to the camera. If the was already opened, close it first.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::open_camera()
{
	std::cout << "JaiGenicamCameraControl open camera" << std::endl;
	J_STATUS_TYPE   retval;
	std::stringstream err_msg;
	std::unique_lock<std::mutex> lock(this->camera_mutex, std::defer_lock);
	lock.lock();
	// Close camera if already open
	if (this->camera_handle != NULL)
	{
		lock.unlock();
		
		retval = this->close_camera();
		if (retval != 0)
		{
			return retval;
		};
		lock.lock();
	};
	retval = J_Camera_Open(this->factory_handle, this->camera_id_s, &this->camera_handle);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not open camera! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("open_camera", "J_Camera_Open", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	else
	{
		std::cout << "Camera open." << std::endl;
	}
	return 0;
}; // JaiGenicamCameraControl::open_camera


/** ===================================================================
Close the connection to the camera. 

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::close_camera()
{
	std::cout << "JaiGenicamCameraControl close camera" << std::endl;
	J_STATUS_TYPE   retval;
	std::stringstream err_msg;
	std::unique_lock<std::mutex> lock(this->camera_mutex);
	if (this->camera_handle != NULL)
	{
		// Stop acquisition of started
		if (this->capture_thread_handle != NULL)
		{
			lock.unlock();
			retval = this->stop_camera_acquisition();
			if (retval != 0)
			{
				return retval;
			};
			lock.lock();
		};
		retval = J_Camera_Close(this->camera_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not close camera!" << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("close_camera", "J_Camera_Close", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		else
		{
			std::cout << "Camera closed." << std::endl;
			this->camera_handle = NULL;
		}			
	}
	return 0;
}; // JaiGenicamCameraControl::close_camera


int JaiGenicamCameraControl::get_camera_list(std::vector<std::string> &camera_list)
{
	std::cout << "JaiGenicamCameraControl get camera list" << std::endl;
	J_STATUS_TYPE   retval;
	bool8_t         has_changed;
	uint32_t        n_cameras;
	int8_t          camera_info_s[J_CAMERA_ID_SIZE];
	uint32_t        size;

	std::stringstream status_stream;
	std::stringstream err_msg;
	GenicamErrorStruct error_data;

	camera_list.clear();

	std::unique_lock<std::mutex> lock(this->camera_mutex, std::defer_lock);
	lock.lock();
	
	//Update camera list
	retval = J_Factory_UpdateCameraList(this->factory_handle, &has_changed);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not update camera list!" << this->get_error_string(retval); 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_UpdateCameraList", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}	
	lock.unlock();

	// Get the number of Cameras
	lock.lock();
	retval = J_Factory_GetNumOfCameras(this->factory_handle, &n_cameras);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Failure getting number of cameras!" << this->get_error_string(retval); 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_GetNumOfCameras", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	else
	{
		status_stream << "Found " << n_cameras << " cameras" << std::endl;
		std::cout << status_stream.str();
	}
	lock.unlock();

	if (n_cameras == 0)
	{
		retval = -1;
		err_msg << "No cameras were found."; 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_GetNumOfCameras", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}

	// Get camera ID
	int8_t   s_camera_info[J_CAMERA_INFO_SIZE];
	uint32_t size_ci;
	std::string info;
	_J_CAMERA_INFO_TYPE id_type;

	for (int i=0; i<n_cameras; i++)
	{
		size_ci = (uint32_t)sizeof(s_camera_info);
		size = (uint32_t)sizeof(camera_info_s);
		lock.lock();
		retval = J_Factory_GetCameraIDByIndex(this->factory_handle, i, camera_info_s, &size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera ID!" << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraIDByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		lock.unlock();

		std::string serial_string;
		
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_MODELNAME;
		size_ci = (uint32_t)sizeof(s_camera_info);
		lock.lock();
		retval = J_Factory_GetCameraInfo(this->factory_handle, camera_info_s, id_type, s_camera_info, &size_ci);
		lock.unlock();
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera model info! " << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}		
		for (int k=0; k<size_ci-1; k++)
		{
			serial_string += (char)s_camera_info[k];
		}

		serial_string += ", ";
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_MANUFACTURER;
		size_ci = (uint32_t)sizeof(s_camera_info);
		lock.lock();
		retval = J_Factory_GetCameraInfo(this->factory_handle, camera_info_s, id_type, s_camera_info, &size_ci);
		lock.unlock();
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera manu info! " << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}		
		for (int k=0; k<size_ci-1; k++)
		{
			serial_string += (char)s_camera_info[k];
		}

		

		serial_string += ", serial: ";
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_SERIALNUMBER;
		size_ci = (uint32_t)sizeof(s_camera_info);
		lock.lock();
		retval = J_Factory_GetCameraInfo(this->factory_handle, camera_info_s, id_type, s_camera_info, &size_ci);
		lock.unlock();
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera serial info! " << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}		
		for (int k=0; k<size_ci-1; k++)
		{
			serial_string += (char)s_camera_info[k];
		}
		
		serial_string += ", ip address: ";
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_IP;
		size_ci = (uint32_t)sizeof(s_camera_info);
		lock.lock();
		retval = J_Factory_GetCameraInfo(this->factory_handle, camera_info_s, id_type, s_camera_info, &size_ci);
		lock.unlock();
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera ip info! " << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}		
		for (int k=0; k<size_ci-1; k++)
		{
			serial_string += (char)s_camera_info[k];
		}

		camera_list.push_back(serial_string);
	}
	return 0;
}; // JaiGenicamCameraControl::get_camera_list


/** ===================================================================
Scan the network interfaces and try to find the correct camera.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::find_camera(std::string serial, int8_t* camera_id_s_p)
{
	std::cout << "JaiGenicamCameraControl find camera" << std::endl;
	J_STATUS_TYPE   retval;
	bool8_t         has_changed;
	uint32_t        n_cameras;
	int8_t          camera_id_s[J_CAMERA_ID_SIZE];
	uint32_t        size;

	std::stringstream status_stream;
	std::stringstream err_msg;
	GenicamErrorStruct error_data;

	std::unique_lock<std::mutex> lock(this->camera_mutex, std::defer_lock);
	lock.lock();
	
	//Update camera list
	retval = J_Factory_UpdateCameraList(this->factory_handle, &has_changed);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not update camera list!" << this->get_error_string(retval); 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_UpdateCameraList", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	else
	{
		if (has_changed == true)
		{
			std::cout << "Camera list updated. New cameras found." << std::endl;
		}
		else
		{
			std::cout << "Camera list updated. No new cameras found." << std::endl;
		}
	}

	// Get the number of Cameras
	retval = J_Factory_GetNumOfCameras(this->factory_handle, &n_cameras);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Failure getting number of cameras!" << this->get_error_string(retval); 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_GetNumOfCameras", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	else
	{
		status_stream << "Found " << n_cameras << " cameras" << std::endl;
//		this->emit_status_message(status_stream.str());
		std::cout << status_stream.str();
	}

	if (n_cameras == 0)
	{
		retval = -1;
		err_msg << "No cameras were found."; 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_GetNumOfCameras", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}

	// Get camera ID
	int8_t   s_camera_info[J_CAMERA_INFO_SIZE];
	uint32_t size_ci;
	std::string target_serial;
	target_serial = serial;
	bool camera_found = false;
	_J_CAMERA_INFO_TYPE id_type;
	if (this->is_ipaddress(target_serial) == true)
	{
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_IP;
	}
	else
	{
		id_type = _J_CAMERA_INFO_TYPE::CAM_INFO_SERIALNUMBER;
	}

	for (int i=0; i<n_cameras; i++)
	{
		std::cout << "--------------------------------------------" << std::endl;

		size_ci = (uint32_t)sizeof(s_camera_info);
		size = (uint32_t)sizeof(camera_id_s);
		retval = J_Factory_GetCameraIDByIndex(this->factory_handle, i, camera_id_s, &size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera ID!" << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraIDByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		else
		{
			std::cout << "Camera " << i << " ID: " << camera_id_s << std::endl;
		}

		retval = J_Factory_GetCameraInfo(this->factory_handle, camera_id_s, id_type, s_camera_info, &size_ci);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not get the camera info!" << this->get_error_string(retval); 
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		else
		{
			status_stream << "Camera " << i << " id: " << s_camera_info << std::endl;
//			this->emit_status_message(status_stream.str());
			std::cout << status_stream.str();
		}
		std::string serial_string;
		for (int k=0; k<size_ci-1; k++)
		{
			serial_string += (char)s_camera_info[k];
		}
		std::cout << "Target is: " << target_serial << ", this camera " << serial_string << std::endl;
		if (target_serial == serial_string)
		{
			camera_found = true;
			std::cout << "Found camera with id " << target_serial << ", index " << i << std::endl;
			break;
		}
	};
	if (camera_found == false)
	{
		// Did not find a camera with correct serial number
		retval = -1;
		err_msg << "Camera id number not found on bus."; 
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("find_camera", "J_Factory_GetCameraInfo", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	};
//	std::copy(std::begin(camera_id_s), std::end(camera_id_s), std::begin(this->camera_id_s));
	std::copy(std::begin(camera_id_s), std::end(camera_id_s), camera_id_s_p);
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::find_camera



/** Iterate through the nodes of the camera, populating the node map structure. This is done to be able to access the data
quickly without interrupting the camera when aquiring images.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::populate_node_map()
{
	uint32_t       n_nodes;
	uint32_t       n_feature_nodes;
	J_STATUS_TYPE  retval;
	NODE_HANDLE    h_node;
	int8_t         s_node_name[256];
	uint32_t       size = 256; 
	J_NODE_TYPE	   node_type;
	GenicamGenericNode sub_node;

	std::cout << "Entering populate_node_map " << std::endl;

	std::stringstream err_msg;

	std::stack<std::string> node_category_stack;
	std::string category_name_string;
	std::string subfeature_name_string;

	node_category_stack.push("Root");

	std::unique_lock<std::mutex> lock(this->camera_mutex);
	
	this->exposuretime_node_name.clear();

	while (node_category_stack.empty() == false)
	{
		// Get next category name:
		category_name_string.assign(node_category_stack.top());		
		node_category_stack.pop();
		
		// Get number of subfeature nodes:
//		std::cout << "name_string " << category_name_string << std::endl;
		retval = J_Camera_GetNumOfSubFeatures(this->camera_handle, (int8_t*)category_name_string.c_str(), &n_feature_nodes);
		if (retval != J_ST_SUCCESS)
		{  
			std::cout << "GetNumOfSubFeatrures error " << this->get_error_string(retval);
			err_msg << this->get_error_string(retval);
			this->set_error_data("populate_node_map", "J_Camera_GetNumOfSubFeatures", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		
//		std::cout << category_name_string << " found " << n_feature_nodes << " sub feature nodes" << std::endl;
		
		for (uint32_t index_f = 0; index_f < n_feature_nodes; ++index_f)
		{
			
			// Get sub feature node handle:
			retval = J_Camera_GetSubFeatureByIndex(this->camera_handle, (int8_t*)category_name_string.c_str(), index_f, &h_node);
			if (retval != J_ST_SUCCESS)
			{  
				std::cout << "J_Camera_GetSubFeatureByIndex error " << this->get_error_string(retval);

				err_msg << this->get_error_string(retval);
				this->set_error_data("populate_node_map", "J_Camera_GetSubFeatureByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);

				return retval;
			}
			// Get node name:
			size = 256;
			retval = J_Node_GetName(h_node, s_node_name, &size);
			if (retval != J_ST_SUCCESS)
			{  
				std::cout << "J_Node_GetName error " << this->get_error_string(retval);
				err_msg << this->get_error_string(retval);
				this->set_error_data("populate_node_map", "J_Node_GetName", err_msg.str(), retval, CameraState::NO_STATE, false);
				return retval;
			}
			subfeature_name_string.assign((char*) s_node_name, size-1);
//			std::cout << category_name_string << " subfeature " << index_f << " name " << subfeature_name_string << std::endl;
			// Get node type:
			retval = J_Node_GetType(h_node, &node_type);
			if (retval != J_ST_SUCCESS)
			{  
				std::cout << "J_Node_GetType error " << this->get_error_string(retval);
				err_msg << this->get_error_string(retval);
				this->set_error_data("populate_node_map", "J_Node_GetType", err_msg.str(), retval, CameraState::NO_STATE, false);
				return retval;
			}
			// std::cout << "Subfeature " << index_f << " type " << node_type << std::endl;
			if (node_type == J_NODE_TYPE::J_ICategory)
			{
				node_category_stack.push(subfeature_name_string);
			}
			else
			{
				lock.unlock();
				retval = this->generate_genericnode_from_name(subfeature_name_string, sub_node);
				lock.lock();
				this->node_map.insert(std::make_pair(subfeature_name_string, sub_node));
			}
			// Check if the node is the exposuretime node
			// Then save that name. We will use it late to check if the camera is alive.
			if (node_type == J_NODE_TYPE::J_IInteger || node_type == J_NODE_TYPE::J_IFloat)
			{
				std::string node_name = subfeature_name_string;
				std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
				int found_pos = node_name.find("exposure");
				if (found_pos != node_name.npos)
				{
					this->exposuretime_node_name = subfeature_name_string;
					std::cout << "Exposuretime node found: " << this->exposuretime_node_name << std::endl;
				};
			};

		}
		
		
	}
	if (this->exposuretime_node_name.empty() == true)
	{
		std::cout << "Exposuretime node not found." << std::endl;
		this->set_error_data("populate_node_map", "exposure_time_node", "Exposuretime node not found", -1, CameraState::UNKNOWN_STATE, false);
		return -1;
	};
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::populate_node_map


int JaiGenicamCameraControl::generate_genericnode_from_name(std::string node_name, GenicamGenericNode &generic_node)
{
		
	J_NODE_TYPE			node_type;
	J_STATUS_TYPE		retval;
	std::stringstream	err_msg;
	int64_t				int_value;
	uint32_t			uint_value;
	std::string			string_value;		
	double				double_value;
	char				char_buffer_p[512];
	uint32_t			char_buffer_size;
	NODE_HANDLE			h_node;

//	GenicamGenericNode generic_node;
	generic_node.name = node_name;

	std::lock_guard<std::mutex> lock(this->camera_mutex);
	retval = J_Camera_GetNodeByName(this->camera_handle, (int8_t*)node_name.c_str(), &h_node);
	if (retval != J_ST_SUCCESS)
	{ 
		err_msg = std::stringstream();
		err_msg <<  node_name << " failure getting name, returned " << this->get_error_string(retval) ;
		std::cout << err_msg.str() << std::endl;
		generic_node.valid = false;
		this->set_error_data("generate_genericnode_from_name", "J_Camera_GetNodeByName", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
		
	char_buffer_size = 512;
	retval = J_Node_GetDescription(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
	if (retval == J_ST_SUCCESS)
	{
		generic_node.description = std::string(char_buffer_p, char_buffer_size-1);	// Size - 1 to remove trailing null termination
	}
	else
	{
		switch (retval)
		{
		case J_ST_GC_ERROR:
			tGenICamErrorInfo gc;
			J_Factory_GetGenICamErrorInfo(&gc);
			err_msg = std::stringstream();
			err_msg << gc.sNodeName << " GC Error: get description returned " << gc.sDescription;
//			std::cout << err_msg.str() << std::endl;
			generic_node.valid = false;
			this->set_error_data("generate_genericnode_from_name", "J_Node_GetDescription", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		default:
			err_msg = std::stringstream();
			err_msg << node_name << " failure getting node description, returned " << this->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			generic_node.valid = false;
			return retval;
		}

	}		

	char_buffer_size = 512;
	retval = J_Node_GetUnit(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
	if (retval == J_ST_SUCCESS)
	{
		generic_node.unit = std::string(char_buffer_p, char_buffer_size-1); // Size - 1 to remove trailing null termination
	}
	else 
	{
		switch (retval)
		{
		case J_ST_GC_ERROR:
			tGenICamErrorInfo gc;
			J_Factory_GetGenICamErrorInfo(&gc);
			err_msg = std::stringstream();
			err_msg << gc.sNodeName << " GC Error: Failure getting node unit, returned " << gc.sDescription;
//			std::cout << err_msg.str() << std::endl;
			generic_node.valid = false;
			this->set_error_data("generate_genericnode_from_name", "J_Node_GetUnit", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
	}

	retval = J_Node_GetType(h_node, &node_type);
	if (retval != J_ST_SUCCESS)
	{
		err_msg = std::stringstream();
		err_msg << node_name << " failure getting type, returned " << this->get_error_string(retval) ;
		std::cout << err_msg.str() << std::endl;
		generic_node.valid = false;
		this->set_error_data("generate_genericnode_from_name", "J_Node_GetType", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
//		std::cout << "Node type " << (int)node_type << std::endl;
	switch (node_type)
	{
		case J_IInteger:
		case J_IRegister:
		case J_IBoolean:
		case J_IIntSwissKnife:
		case J_IIntReg:
		{
			// It was an integer value
//				std::cout << "Node type integer" << std::endl;
			generic_node.valid = true;
			generic_node.type = J_NODE_TYPE::J_IInteger;
			// Get actual value:
			retval = J_Node_GetValueInt64(h_node, true, &int_value);
			
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg = std::stringstream();
					err_msg << gc.sNodeName << " GC Error: Failure getting node int value, returned " << gc.sDescription;
//					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name <<  " failure getting int value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}			
			generic_node.value_i = int_value;
//				std::cout << "Current value: " << int_value << std::endl;
			// Get min value:
			retval = J_Node_GetMinInt64(h_node, &int_value);
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg = std::stringstream();
					err_msg << gc.sNodeName << " GC Error: Failure getting node int min value, returned " << gc.sDescription;
//					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMinInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name << " failure getting int min value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMinInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}
			
			generic_node.min_value_i = int_value;
			// Get max value:
			retval = J_Node_GetMaxInt64(h_node, &int_value);
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg = std::stringstream();
					err_msg << gc.sNodeName << " GC Error: Failure getting node int max value, returned " << gc.sDescription;
//					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMaxInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name << " failure getting int max value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMaxInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}
			generic_node.max_value_i = int_value;
			
			break;
		}

		case J_IFloat:
		case J_ISwissKnife:
		{
			// It was a float value
//				std::cout << "Node type float" << std::endl;
			generic_node.valid = true;
			generic_node.type = J_NODE_TYPE::J_IFloat;
			// Get actual value:
			retval = J_Node_GetValueDouble(h_node, true, &double_value);
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					err_msg = std::stringstream();
//					err_msg <<  "GC Error: Failure getting node double value ";
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg << gc.sNodeName << " double value, returned " << gc.sDescription;
//					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name << " failure getting double value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}
			generic_node.value_d = double_value;
//				std::cout << "Current value: " << double_value << std::endl;
			// Get min value:
			retval = J_Node_GetMinDouble(h_node, &double_value);
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					err_msg = std::stringstream();
//					err_msg <<  "GC Error: Failure getting node double min ";
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg << gc.sNodeName << " double min value, returned " << gc.sDescription;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMinDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name << " failure getting double min value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMinDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}
			
			generic_node.min_value_d = double_value;
			// Get max value:
			retval = J_Node_GetMaxDouble(h_node, &double_value);
			
			if (retval != J_ST_SUCCESS)
			{
				switch (retval)
				{
				case J_ST_GC_ERROR:
					err_msg = std::stringstream();
//					err_msg <<  "GC Error: Failure getting node double max ";
					tGenICamErrorInfo gc;
					J_Factory_GetGenICamErrorInfo(&gc);
					err_msg << gc.sNodeName << " double max value, returned " << gc.sDescription;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMaxDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
					break;
				case J_ST_INVALID_PARAMETER:
					generic_node.valid = false;
					break;
				default:
					err_msg = std::stringstream();
					err_msg << node_name << " failure getting double max value, returned " << this->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					generic_node.valid = false;
					this->set_error_data("generate_genericnode_from_name", "J_Node_GetMaxDouble", err_msg.str(), retval, CameraState::NO_STATE, false);
				}
			}
			generic_node.max_value_d = double_value;			
			
			break;
		}

		case J_IEnumeration:
		case J_IEnumEntry:
			{
				// It was an enumeration value
//				std::cout << "Node type enum: " << generic_node.name << std::endl;
				generic_node.valid = true;
				generic_node.type = J_NODE_TYPE::J_IEnumeration;
				// Get actual value:
				retval = J_Node_GetValueInt64(h_node, true, &int_value);				
				if (retval != J_ST_SUCCESS)
				{
					switch (retval)
					{
					case J_ST_GC_ERROR:
						err_msg = std::stringstream();
						tGenICamErrorInfo gc;
						J_Factory_GetGenICamErrorInfo(&gc);						
						err_msg << gc.sNodeName << " enum value, returned " << gc.sDescription;
						generic_node.valid = false;
						this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
						break;
					case J_ST_INVALID_PARAMETER:
						generic_node.valid = false;
						break;
					default:
						err_msg = std::stringstream();
						err_msg << node_name << " failure getting enum value, returned " << this->get_error_string(retval) ;
						std::cout << err_msg.str() << std::endl;
						generic_node.valid = false;
						this->set_error_data("generate_genericnode_from_name", "J_Node_GetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
					}
				}
				generic_node.value_i = int_value;
//				std::cout << "Current value: " << int_value << std::endl;
					
				uint32_t num_enum = 0;
				retval = J_Node_GetNumOfEnumEntries(h_node, &num_enum);
				
				if (retval != J_ST_SUCCESS)
				{
					switch (retval)
					{
					case J_ST_GC_ERROR:
						tGenICamErrorInfo gc;
						J_Factory_GetGenICamErrorInfo(&gc);
						err_msg = std::stringstream();
						err_msg << gc.sNodeName << " number of enum entries, returned " << gc.sDescription;
						this->set_error_data("generate_genericnode_from_name", "J_Node_GetNumOfEnumEntries", err_msg.str(), retval, CameraState::NO_STATE, false);
						generic_node.valid = false;
						break;
					case J_ST_INVALID_PARAMETER:
						generic_node.valid = false;
						break;
					default:
						err_msg = std::stringstream();
						err_msg << node_name << " failure getting number of enum entries, returned " << this->get_error_string(retval) ;
						std::cout << err_msg.str() << std::endl;
						generic_node.valid = false;
						this->set_error_data("generate_genericnode_from_name", "J_Node_GetNumOfEnumEntries", err_msg.str(), retval, CameraState::NO_STATE, false);
					}
				}
//				std::cout << "Number of enum entries: " << num_enum << std::endl;

				NODE_HANDLE enum_entry;
				J_NODE_ACCESSMODE access_mode;
				std::stringstream	desc_stream;
				desc_stream << generic_node.description << std::endl << std::endl << "Valid enum values: " << std::endl;
				generic_node.enum_names.clear();
				for (uint32_t j=0; j<num_enum; j++)
				{
					retval = J_Node_GetEnumEntryByIndex(h_node, j, &enum_entry);
					
					if (retval != J_ST_SUCCESS)
					{
						switch (retval)
						{
						case J_ST_GC_ERROR:
							err_msg <<  "GC Error: Failure enum entry ";
							tGenICamErrorInfo gc;
							J_Factory_GetGenICamErrorInfo(&gc);
							err_msg = std::stringstream();
							err_msg << gc.sNodeName << " enum entry, returned " << gc.sDescription;
							generic_node.valid = false;
							this->set_error_data("generate_genericnode_from_name", "J_Node_GetEnumEntryByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);
							break;
						case J_ST_INVALID_PARAMETER:
							generic_node.valid = false;
							break;
						default:
							err_msg = std::stringstream();
							err_msg << node_name << " failure getting enum entry, returned " << this->get_error_string(retval) ;
							std::cout << err_msg.str() << std::endl;
							generic_node.valid = false;
							this->set_error_data("generate_genericnode_from_name", "J_Node_GetEnumEntryByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);
						}
					}
					else
					{
						char_buffer_size = 512;
						retval = J_Node_GetName(enum_entry, (int8_t*)char_buffer_p, &char_buffer_size);						
						if (retval != J_ST_SUCCESS)
						{
							switch (retval)
							{
							case J_ST_GC_ERROR:
								tGenICamErrorInfo gc;
								J_Factory_GetGenICamErrorInfo(&gc);
								err_msg = std::stringstream();
								err_msg << gc.sNodeName << " enum entry node name, returned " << gc.sDescription;
								generic_node.valid = false;
								this->set_error_data("generate_genericnode_from_name", "J_Node_GetName", err_msg.str(), retval, CameraState::NO_STATE, false);
								break;
							case J_ST_INVALID_PARAMETER:
								generic_node.valid = false;
								break;
							default:
								err_msg = std::stringstream();
								err_msg << node_name << " failure getting enum entry node name, returned " << this->get_error_string(retval) ;
								std::cout << err_msg.str() << std::endl;
								generic_node.valid = false;
								this->set_error_data("generate_genericnode_from_name", "J_Node_GetName", err_msg.str(), retval, CameraState::NO_STATE, false);
							}
						}
						else
						{
//							std::cout << "Enum entry " << j << ": " << char_buffer_p << std::endl;
						}
						retval = J_Node_GetAccessMode(enum_entry, &access_mode);						
						if (retval != J_ST_SUCCESS)
						{
							switch (retval)
							{
							case J_ST_GC_ERROR:
								err_msg <<  "GC Error: Failure enum entry node access mode ";
								tGenICamErrorInfo gc;
								J_Factory_GetGenICamErrorInfo(&gc);
								err_msg = std::stringstream();
								err_msg << gc.sNodeName << " enum entry node access mode, returned " << gc.sDescription;
								generic_node.valid = false;
								this->set_error_data("generate_genericnode_from_name", "J_Node_GetAccessMode", err_msg.str(), retval, CameraState::NO_STATE, false);
								break;
							case J_ST_INVALID_PARAMETER:
								generic_node.valid = false;
								break;
							default:
								err_msg = std::stringstream();
								err_msg << node_name << " failure getting enum entry node access mode, returned " << this->get_error_string(retval) ;
								std::cout << err_msg.str() << std::endl;
								generic_node.valid = false;
								this->set_error_data("generate_genericnode_from_name", "J_Node_GetAccessMode", err_msg.str(), retval, CameraState::NO_STATE, false);
							}
						}
						else
						{								
							switch (access_mode)
							{
							case _J_NODE_ACCESSMODE_TYPE::RO:
							case _J_NODE_ACCESSMODE_TYPE::RW:
							case _J_NODE_ACCESSMODE_TYPE::WO:
								retval = J_Node_GetEnumEntryValue(enum_entry, &int_value);
								std::string entry_map_string = char_buffer_p+9+2+generic_node.name.length();
								desc_stream << entry_map_string << ", " << std::endl;
//								std::cout << "Cut enum name: " << entry_map_string << std::endl;
								generic_node.enum_names.push_back(entry_map_string);
								generic_node.enum_entry_map[entry_map_string] = int_value;
								generic_node.enum_value_map[int_value] = entry_map_string;
							}								
						}
					} 
				} // For enum entries
				generic_node.description = desc_stream.str();
				break;
			}

		case J_IStringReg:			
		default:
			generic_node.valid = false;
	}; // switch (node_type)
//	std::cout << "Node description: " << generic_node.description << std::endl;
	return 0;
};	// JaiGenicamCameraControl::generate_genericnode_from_name


/** Write node data to camera
*/
int JaiGenicamCameraControl::set_node_to_camera(GenicamGenericNode generic_node)
{
	J_STATUS_TYPE		retval;
	std::stringstream	err_msg;

	bool debug_output = true;

	if (debug_output == true)
	{
		std::cout << "Setting camera node " << generic_node.name;
	}
	std::lock_guard<std::mutex> lock(this->camera_mutex);
	switch (generic_node.type)
	{
	case J_NODE_TYPE::J_IFloat:
		retval = J_Camera_SetValueDouble(this->camera_handle, (int8_t*) generic_node.name.c_str(), generic_node.value_d);
		if (debug_output == true)
		{
			std::cout << " double to " << generic_node.value_d << ", return code " << retval << std::endl;
		}
		break;
	case J_NODE_TYPE::J_IInteger:
		retval = J_Camera_SetValueInt64(this->camera_handle, (int8_t*) generic_node.name.c_str(), generic_node.value_i);
		if (debug_output == true)
		{
			std::cout << " int to " << generic_node.value_i << ", return code " << retval << std::endl;
		}
		break;
	case J_NODE_TYPE::J_IEnumeration:
		retval = J_Camera_SetValueInt64(this->camera_handle, (int8_t*) generic_node.name.c_str(), generic_node.value_i);
		if (debug_output == true)
		{
			std::cout << " enum int to " << generic_node.value_i << ", return code " << retval << std::endl;
		}
		break;
	case J_NODE_TYPE::J_IStringReg:
		retval = J_Camera_SetValueString(this->camera_handle, (int8_t*) generic_node.name.c_str(), (int8_t*)generic_node.value_s.c_str());
		if (debug_output == true)
		{
			std::cout << " string to " << generic_node.value_s << ", return code " << retval << std::endl;
		}
		break;
	default:
		retval = J_ST_INVALID_PARAMETER;
	};
	if (retval != J_ST_SUCCESS)
	{		
		switch (retval)
		{
		case J_ST_GC_ERROR:
			tGenICamErrorInfo gc;
			J_Factory_GetGenICamErrorInfo(&gc);
			err_msg << gc.sNodeName << " GC Error: Failure setting camera node, returned " << gc.sDescription;
			if (debug_output == true)
			{
				std::cout << err_msg.str() << std::endl;
			}
			this->set_error_data("set_node_to_camera", "J_Camera_SetValueXXX", err_msg.str(), retval, CameraState::NO_STATE, false);
			break;

		default:
			err_msg << generic_node.name << " failure setting camera node, returned " << this->get_error_string(retval) ;
			if (debug_output == true)
			{
				std::cout << err_msg.str() << std::endl;
			}
			this->set_error_data("set_node_to_camera", "J_Camera_SetValueXXX", err_msg.str(), retval, CameraState::NO_STATE, false);
		}
		return retval;
	};
	return 0;
}; // JaiGenicamCameraControl::set_node_to_camera


/** Disable nodes in list off_names with auto in the name to be able to adjust the values. 
E.g. off_names="gain" -> GainAuto to Off
*/
int JaiGenicamCameraControl::disable_auto_nodes()
{
	bool debug_output = false;

	if (debug_output){
		std::cout << "JaiGenicamCameraControl::disable_auto_nodes" << std::endl;}
		
	J_STATUS_TYPE		retval;
	J_NODE_TYPE			node_type;
	std::stringstream	err_msg;
	int64_t				int_value;
	uint32_t			n_nodes;
	std::string				node_name;		
	std::string				auto_name = "auto";
	char				char_buffer_p[512];
	uint32_t			char_buffer_size;
	NODE_HANDLE			h_node;
	std::size_t			found_pos;
	std::vector<std::string>		off_names;
	off_names.push_back("exposure");
	off_names.push_back("gain");

	retval = J_Camera_GetNumOfNodes(this->camera_handle, &n_nodes);
	if (retval != J_ST_SUCCESS)
	{ 
		err_msg <<  "Failure getting number of nodes, returned " << this->get_error_string(retval);
		if (debug_output){
			std::cout << err_msg.str() << std::endl; }
		this->set_error_data("disable_auto_nodes", "J_Camera_GetNumOfNodes", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	if (debug_output){
		std::cout << "Found " << n_nodes << " nodes." << std::endl; }

	for (int i_node=0; i_node < n_nodes; i_node++)
	{
		retval = J_Camera_GetNodeByIndex(this->camera_handle, i_node, &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node index " << i_node << ", returned " << this->get_error_string(retval);
			if (debug_output){
				std::cout << err_msg.str() << std::endl; }
			this->set_error_data("disable_auto_nodes", "J_Camera_GetNodeByIndex", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		char_buffer_size = 512;
		retval = J_Node_GetName(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node name for index " << i_node << ", returned " << this->get_error_string(retval) ;
			if (debug_output){
				std::cout << err_msg.str() << std::endl; }
			this->set_error_data("disable_auto_nodes", "J_Node_GetName", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		node_name = char_buffer_p;
		std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
		found_pos = node_name.find(auto_name);
		if (found_pos != node_name.npos)
		{
			if (debug_output){
				std::cout << "Found name with auto: " << node_name << "." << std::endl; }
			for (auto & off_name : off_names)
			{
				found_pos = node_name.find(off_name);
				if (found_pos != node_name.npos)
				{						
					J_Node_GetType(h_node, &node_type);
					if (node_type == J_IEnumeration)
					{
						if (debug_output){
							std::cout << "Turning off." << std::endl; }
						int_value = 0;
						retval = J_Node_SetValueInt64(h_node, true, int_value);
						if (retval != J_ST_SUCCESS)
						{ 
							err_msg <<  "Failure turning off node " << node_name << ", returned " << this->get_error_string(retval) ;
							if (debug_output){
								std::cout << err_msg.str() << std::endl; }
							this->set_error_data("disable_auto_nodes", "J_Node_SetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
							return retval;
						}
					}
				}
			}
		}
	}

	return 0;
}; // JaiGenicamCameraControl::disable_auto_nodes



/** ===================================================================
Start camera acquisition. Open capture stream.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::start_camera_acquisition()
{
	std::cout << "Starting aquisition" << std::endl;

	J_STATUS_TYPE   retval;
	std::stringstream err_msg;

	// Close old stream if active
	if(this->capture_thread_handle != NULL)
	{				
		retval = this->stop_camera_acquisition();
		if (retval != 0)
		{
			std::cout << "start_camera_acquisition: Error stopping aquisition. " << this->get_error_string(retval) << std::endl;			
			return retval;
		};
	};
	// Make sure the camera is open
	if (this->camera_handle == NULL)
	{
		retval = this->open_camera();
		if (retval != 0)
		{
			std::cout << "start_camera_acquisition: Error opening camera. " << this->get_error_string(retval) << std::endl;
			return retval;
		};
	};

	std::unique_lock<std::mutex> lock(this->camera_mutex);	

	
	// Get the pixelformat from the camera
	uint64_t jaiPixelFormat = 0;
	int64_t remote_pixelformat;
	retval = this->get_node_value("PixelFormat", remote_pixelformat);

	retval = J_Image_Get_PixelFormat(this->camera_handle, remote_pixelformat, &jaiPixelFormat);
	if (retval != J_ST_SUCCESS) {
		err_msg << "Could not get pixelformat! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_camera_acquisition", "J_Image_Get_PixelFormat", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	};

	// Calculate number of bits (not bytes) per pixel using macro
	int bpp = J_BitsPerPixel(jaiPixelFormat);
	int64_t image_width;
	int64_t image_height;
	retval = this->get_node_value("Width", image_width);
	if (retval != J_ST_SUCCESS) {
		err_msg << "Could not get image width! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		return retval;
	};
	retval = this->get_node_value("Height", image_height);
	if (retval != J_ST_SUCCESS) {
		err_msg << "Could not get image height! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		return retval;
	};

	std::cout << "Image parameters: " << std::endl << "  height..." << image_height << std::endl << "   width..." << image_width
		<< std::endl << "     bpp..." << bpp << std::endl << "    size..." << (image_height*image_width*bpp)/8 << std::endl;

	// Open stream
	retval = J_Image_OpenStream(this->camera_handle, 0, 
		reinterpret_cast<J_IMG_CALLBACK_OBJECT>(this), 
		reinterpret_cast<J_IMG_CALLBACK_FUNCTION>(&JaiGenicamCameraControl::capture_stream_callback), 
		&this->capture_thread_handle, (image_height*image_width*bpp)/8);
	if (retval != J_ST_SUCCESS) {
		err_msg << "Could not open stream! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_camera_acquisition", "J_Image_OpenStream", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	};
	std::cout << "Stream open " << std::endl;

	// Start Acquisition
	retval = J_Camera_ExecuteCommand(this->camera_handle, NODE_NAME_ACQSTART);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not start aquisition! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_camera_acquisition", "J_Camera_ExecuteCommand", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	std::cout << "Acquisition started" << std::endl;
	lock.unlock();
		
	return 0;
}; // JaiGenicamCameraControl::start_camera_acquisition


/** ===================================================================
Stop camera acquisition. Close capture stream.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::stop_camera_acquisition()
{
	std::cout << "JaiGenicamCameraControl::stop_camera_acquisition: Stopping aquisition... " << std::endl;

	J_STATUS_TYPE retval;
	std::stringstream err_msg;
	

	// Lock the mutex to keep start_capture to execute while we are still stopping
	std::unique_lock<std::mutex> lock(this->camera_mutex);	
	// Stop Acquisition
	if (this->camera_handle != NULL) 
	{
		retval = J_Camera_ExecuteCommand(this->camera_handle, NODE_NAME_ACQSTOP);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not Stop Acquisition! " << this->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			this->set_error_data("stop_camera_acquisition", "J_Camera_ExecuteCommand", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		};
		std::cout << "JaiGenicamCameraControl::stop_camera_acquisition: Acquisition stopped..." << std::endl;
	};

	if(this->capture_thread_handle != NULL)
	{
		lock.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		lock.lock();

		// Close stream
		retval = J_Image_CloseStream(this->capture_thread_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not close Stream!! " << this->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			this->set_error_data("stop_camera_acquisition", "J_Image_CloseStream", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		this->capture_thread_handle = NULL;
		std::cout << "JaiGenicamCameraControl::stop_camera_acquisition: Stream closed. Success." << std::endl;
	}
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::stop_camera_acquisition
