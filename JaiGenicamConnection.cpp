#include "JaiGenicamConnection.h"
#include <chrono>
#include "JaiGenicamDS.h"
#include <iostream>
#include <stdexcept>
#include <math.h>
#include <thread>

/* ----------------------------------------
	JaiGenicamConnection implementation
-------------------------------------------*/

namespace JaiGenicamConnection_ns
{

	JaiGenicamConnection::JaiGenicamConnection(::JaiGenicamDS_ns::JaiGenicamDS* tango_ds_class, std::string camera_serial)
	{		

		std::cout << "Creating JaiGenicamConnection object" << std::endl;

		this->tango_ds_class = tango_ds_class;
		this->tango_state = tango_ds_class->get_state();

		this->camera_serial = camera_serial;
		this->camera_handle = NULL;
		this->factory_handle = NULL;
		this->capture_thread_handle = NULL;

		this->image_buffer.pImageBuffer = NULL;
		this->frame_counter = 0;
		this->fps_vector.clear();
		
		std::cout << "Nodes" << std::endl;
		GenicamNode<uint64_t> gn = { "GainRaw",	// name
									 "int",		// type
									 "",		// unit
									 "",		// description
									 0,			// value
									 0,			// min_value
									 0,			// max_value
									 false };	// valid
		this->gain_node = gn;
		std::cout << "Gain node" << std::endl;
		GenicamNode<double> en = { "ExposureTime", "double", "", "", 0, 0, 0, false };
		this->exposuretime_node = en;
		std::cout << "Exposuretime node" << std::endl;
		GenicamNode<uint64_t> fn = { "JAIAcquisitionFrameRate", "enum", "", "", 0, 0, 0, false };
		this->framerate_node = fn;
		std::cout << "Framerate node" << std::endl;
		GenicamNode<uint64_t> wn = { "Width", "int", "", "", 0, 0, 0, false };
		this->imagewidth_node = wn;
		std::cout << "Width node" << std::endl;
		GenicamNode<uint64_t> hn = { "Height", "int", "", "", 0, 0, 0, false };
		this->imageheight_node = hn;
		std::cout << "Height node" << std::endl;
		GenicamNode<uint64_t> pfn = { "PixelFormat",	// name
									 "enum",		// type
									 "",		// unit
									 "",		// description
									 0,			// value
									 0,			// min_value
									 0,			// max_value
									 false };	// valid
		this->pixelformat_node = pfn;

		this->error_data.calling_function = "";
		this->error_data.calling_state = "";
		this->error_data.error_message = "";
		this->error_data.retval = J_ST_SUCCESS;

		std::cout << "State" << std::endl;
		this->jai_genicam_state = JaiGenicamDisconnectedState::Instance();
		std::cout << "State instance: " << this->jai_genicam_state << std::endl;
	}


	JaiGenicamConnection::~JaiGenicamConnection(void)
	{
		std::cout << "JaiGenicamConnection destructor" << std::endl;
		close_camera();
		close_factory();
//		delete this->jai_genicam_state;
	}

	void JaiGenicamConnection::change_state(JaiGenicamState* new_state)
	{
		std::cout << "Changing state." << std::endl;
		this->jai_genicam_state = new_state;
		std::cout << "New state object." << std::endl;
		this->jai_genicam_state->enter(this);
	};

	void JaiGenicamConnection::start_capture()
	{
		this->jai_genicam_state->start_capture(this);
	};

	void JaiGenicamConnection::stop_capture()
	{
		this->jai_genicam_state->stop_capture(this);
	};

	void JaiGenicamConnection::run()
	{
		this->jai_genicam_state->run(this);
	};

	void JaiGenicamConnection::connect()
	{
		this->jai_genicam_state->connect(this);
	};

	void JaiGenicamConnection::disconnect()
	{
		this->jai_genicam_state->disconnect(this);
	};

	void JaiGenicamConnection::close_camera()
	{
		std::cout << "GenicamConnection::close_camera" << std::endl;
		this->jai_genicam_state->close_camera(this);
	};

	void JaiGenicamConnection::close_factory()
	{
		std::cout << "GenicamConnection::close_factory" << std::endl;
		this->jai_genicam_state->close_factory(this);
	};

	int JaiGenicamConnection::get_gain(double* value_p)
	{
//		std::cout << "GenicamConnection::get_gain" << std::endl; 
		return this->jai_genicam_state->get_gain(this, value_p);
	};

	int JaiGenicamConnection::get_image(uint32_t* width_p, uint32_t* height_p, uint16_t* image_p)
	{
//		std::cout << "GenicamConnection::get_image" << std::endl; 
		return this->jai_genicam_state->get_image(this, width_p, height_p, image_p);
	};

	int JaiGenicamConnection::get_image_info(J_tIMAGE_INFO* aq_image_info_p)
	{
//		std::cout << "GenicamConnection::get_image_info" << std::endl; 
		return this->jai_genicam_state->get_image_info(this, aq_image_info_p);
	}

	int JaiGenicamConnection::get_framecounter(int64_t* value_p)
	{
//		std::cout << "GenicamConnection::get_framecounter" << std::endl; 
		return this->jai_genicam_state->get_framecounter(this, value_p);
	}

	int JaiGenicamConnection::get_framerate(double* value_p)
	{
//		std::cout << "GenicamConnection::get_framerate" << std::endl; 
		return this->jai_genicam_state->get_framerate(this, value_p);
	}


	template<typename T>
	int JaiGenicamConnection::get_node(GenicamNode<T>* node)
	{
//		std::cout << "GenicamConnection::get_node" << std::endl;
		return this->jai_genicam_state->get_node(this, node);
	}
	
	int JaiGenicamConnection::get_node_value(std::string name, double* value_p)
	{
//		std::cout << "GenicamConnection::get_node_value" << std::endl;
		return this->jai_genicam_state->get_node_value(this, name, value_p);
	}

	int JaiGenicamConnection::get_node_value(std::string name, int64_t* value_p)
	{
//		std::cout << "GenicamConnection::get_node_value" << std::endl;
		return this->jai_genicam_state->get_node_value(this, name, value_p);
	}
			
	int JaiGenicamConnection::set_node_value(std::string name, double value)
	{
//		std::cout << "GenicamConnection::set_node_value" << std::endl;
		return this->jai_genicam_state->set_node_value(this, name, value);

	}

	int JaiGenicamConnection::get_node_type(std::string name, std::string* type)
	{
//		std::cout << "GenicamConnection::get_node_type" << std::endl;
		return this->jai_genicam_state->get_node_type(this, name, type);
	}

	int JaiGenicamConnection::get_node_info(std::string name, GenicamGenericNode* generic_node_p)
	{
//		std::cout << "GenicamConnection::get_node_info" << std::endl;
		return this->jai_genicam_state->get_node_info(this, name, generic_node_p);
	}



	void __stdcall JaiGenicamConnection::capture_stream_callback(J_tIMAGE_INFO * aq_imageinfo_p)
	{
		J_STATUS_TYPE   retval;


		std::unique_lock<std::mutex> lock(this->camera_mtx);
			double frame_time;
			frame_time = (std::chrono::duration_cast<std::chrono::milliseconds>
				(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;

			// The state of the buffer is confirmed.

			if(this->image_buffer.pImageBuffer != NULL)
			{
				// When the buffer already exists, and the size is different:
				if((this->image_buffer.iSizeX != aq_imageinfo_p->iSizeX) || (this->image_buffer.iSizeY != aq_imageinfo_p->iSizeY))
				{
					// Abandons the buffer.
					J_Image_Free(&this->image_buffer);
					this->image_buffer.pImageBuffer = NULL;
				}
			}

			// Allocates it when there is no buffer.
			if(this->image_buffer.pImageBuffer == NULL)
				J_Image_Malloc(aq_imageinfo_p, &this->image_buffer);


			// The image making is done for the picture processing.
			retval = J_Image_FromRawToImage(aq_imageinfo_p, &this->image_buffer);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not convert to RAW! " << this->get_error_string(retval) << std::endl;
			};
/*
			std::copy(aq_imageinfo_p->pImageBuffer, 
					aq_imageinfo_p->pImageBuffer + aq_imageinfo_p->iSizeX*aq_imageinfo_p->iSizeY,
					this->image_buffer.pImageBuffer);
*/
			// Update frame counter
			this->frame_counter += 1;

			// Update fps_vector
			if (fps_vector.size() > 10) 
			{
				fps_vector.erase(fps_vector.begin());
			}
			fps_vector.push_back(frame_time);
			
			lock.unlock();
    
	}

	std::string JaiGenicamConnection::get_error_string(J_STATUS_TYPE error)
	{
		std::string error_msg = std::to_string(error);

		switch(error)
		{
		case J_ST_INVALID_BUFFER_SIZE:	error_msg += ": Invalid buffer size ";					break;
		case J_ST_INVALID_HANDLE:		error_msg += ": Invalid handle ";		                break;
		case J_ST_INVALID_ID:			error_msg += ": Invalid ID ";			                break;
		case J_ST_ACCESS_DENIED:		error_msg += ": Access denied ";		                break;
		case J_ST_NO_DATA:				error_msg += ": No data ";								break;
		case J_ST_ERROR:				error_msg += ": Generic error ";		                break;
		case J_ST_INVALID_PARAMETER:	error_msg += ": Invalid parameter ";	                break;
		case J_ST_TIMEOUT:				error_msg += ": Timeout ";								break;
		case J_ST_INVALID_FILENAME:		error_msg += ": Invalid file name ";	                break;
		case J_ST_INVALID_ADDRESS:		error_msg += ": Invalid address ";						break;
		case J_ST_FILE_IO:				error_msg += ": File IO error ";		                break;
		case J_ST_GC_ERROR:				error_msg += ": GenICam error ";		                break;
		case J_ST_VALIDATION_ERROR:		error_msg += ": Settings File Validation Error ";		break;
		case J_ST_VALIDATION_WARNING:	error_msg += ": Settings File Validation Warning ";		break;
		}

		return error_msg;
	}


	/* ---------------------------------
		JaiGenicamState implementation
	------------------------------------*/

	void JaiGenicamState::change_state( JaiGenicamConnection* c, JaiGenicamState* s)
	{
		c->change_state(s);
	};

	int JaiGenicamState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering new state" << std::endl;
		return 0;
	}


	int JaiGenicamState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

	int JaiGenicamState::close_camera(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamState close camera" << std::endl;
		J_STATUS_TYPE   retval;
		if (wrapper->camera_handle != NULL)
		{
			retval = J_Camera_Close(wrapper->camera_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not close camera! " << wrapper->get_error_string(retval) << std::endl;
			}
			else
			{
				std::cout << "Camera closed." << std::endl;
				wrapper->camera_handle = NULL;
			}
			std::cout << "Changing state to disconnected." << std::endl;
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
		}
		return 0;
	};

	int JaiGenicamState::close_factory(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamState close factory" << std::endl;
		return -1;
	
	};

	int JaiGenicamState::disconnect(JaiGenicamConnection* wrapper)
	{
		std::cout << "Disconnecting." << std::endl;
		this->stop_capture(wrapper);
		this->close_camera(wrapper);
		//close_factory(wrapper);
		//change_state(wrapper, JaiGenicamDisconnectedState::Instance());
		return -1;
	}

	int JaiGenicamState::connect(JaiGenicamConnection* wrapper)
	{
		return -1;
	}

	int JaiGenicamState::start_capture(JaiGenicamConnection* wrapper)
	{
		return -1;
	}

	int JaiGenicamState::stop_capture(JaiGenicamConnection* wrapper)
	{
		return -1;
	}

	int JaiGenicamState::get_gain(JaiGenicamConnection* wrapper, double* value_p)
	{
		*value_p = (double)wrapper->gain_node.value;
		std::cout << "get_gain value: " << *value_p << std::endl;

		if (wrapper->gain_node.valid == true)
		{
			return 0;
		}
		else
		{
			return -1;
		}
	}

	int JaiGenicamState::get_image_info(JaiGenicamConnection* wrapper, J_tIMAGE_INFO* aq_image_info_p)
	{
		int retval = -1;
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);		
			if (wrapper->image_buffer.pImageBuffer != NULL)
			{
				aq_image_info_p->iImageSize = wrapper->image_buffer.iImageSize;
				aq_image_info_p->iMissingPackets = wrapper->image_buffer.iMissingPackets;
				aq_image_info_p->iOffsetX = wrapper->image_buffer.iOffsetX;
				aq_image_info_p->iOffsetY = wrapper->image_buffer.iOffsetY;
				aq_image_info_p->iPixelType = wrapper->image_buffer.iPixelType;
				aq_image_info_p->iQueuedBuffers = wrapper->image_buffer.iQueuedBuffers;
				aq_image_info_p->iSizeX = wrapper->image_buffer.iSizeX;
				aq_image_info_p->iSizeY = wrapper->image_buffer.iSizeY;
				aq_image_info_p->iTimeStamp = wrapper->image_buffer.iTimeStamp;
				aq_image_info_p->iAnnouncedBuffers = wrapper->image_buffer.iAnnouncedBuffers;
				aq_image_info_p->iAwaitDelivery = wrapper->image_buffer.iAwaitDelivery;
				aq_image_info_p->pImageBuffer = NULL;
				retval = 0;
//				std::cout << "Image size: " << aq_image_info_p->iImageSize << std::endl;
			};

			lock.unlock();
		return retval;

	}

	int JaiGenicamState::get_framecounter(JaiGenicamConnection* wrapper, int64_t* value_p)
	{
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);
			*value_p = wrapper->frame_counter;
			lock.unlock();
		return 0;
	}

	int JaiGenicamState::get_framerate(JaiGenicamConnection* wrapper, double* value_p)
	{
		double current_time = (std::chrono::duration_cast<std::chrono::milliseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
		double fps_mean = 0;
		double dt_last = 0;
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);
			
			if (wrapper->fps_vector.size() > 1) {
				double dt_mean = (wrapper->fps_vector.back() - wrapper->fps_vector.front()) / (wrapper->fps_vector.size() - 1);

				dt_last = current_time - wrapper->fps_vector.back();
				if (dt_last > 2*dt_mean)
				{
					// We have a substantial delay without getting a new frame, adjust dt
					dt_mean = (current_time - wrapper->fps_vector.front()) / (wrapper->fps_vector.size());
				}
				fps_mean = 1 / dt_mean;		
			}
			*value_p = fps_mean;
			lock.unlock();
		return 0;		
	}


	int JaiGenicamState::get_image(JaiGenicamConnection* wrapper, uint32_t* width_p, uint32_t* height_p, uint16_t* image_p)
	{
		int retval = -1;
//		std::cout << "Entering JaiGenicamState::get_image" << std::endl;
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);		
			if (wrapper->image_buffer.pImageBuffer != NULL)
			{			
				*width_p = wrapper->image_buffer.iSizeX;
				*height_p = wrapper->image_buffer.iSizeY;
				// Check if there are 2 bytes per pixel or 1 byte per pixel:
				if (wrapper->image_buffer.iImageSize / (wrapper->image_buffer.iSizeX * wrapper->image_buffer.iSizeY) < 2)
				{
					// 1 Bpp so do a copy
					std::copy(wrapper->image_buffer.pImageBuffer, 
					wrapper->image_buffer.pImageBuffer+(*width_p)*(*height_p)-1,
					image_p);					
				}
				else
				{
					// 1 Bpp so first reinterpret_cast to uint16_t before copy
					uint16_t* tmp_image_p = reinterpret_cast<uint16_t*>(wrapper->image_buffer.pImageBuffer);
					std::copy(tmp_image_p, 
					tmp_image_p+(*width_p)*(*height_p)-1,
					image_p);	
				}


				retval = 0;
			};
			lock.unlock();
		return retval;
	};

	template<typename T>
	int JaiGenicamState::get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node)
	{
		return -1;
	}

	int JaiGenicamState::get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p)
	{
		return -1;
	}
	
	int JaiGenicamState::get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p)
	{
		return -1;
	}

	int JaiGenicamState::set_node_value(JaiGenicamConnection* wrapper, std::string name, double value)
	{
		return -1;
	}

	int JaiGenicamState::get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type)
	{
		return -1;
	}

	int JaiGenicamState::get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p)
	{
		return -1;
	}

		



	/* -----------------------------------------------
		JaiGenicamDisconnectedState implementation
	--------------------------------------------------*/
	JaiGenicamDisconnectedState* JaiGenicamDisconnectedState::_instance = NULL;

	int JaiGenicamDisconnectedState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering DISCONNECTED state" << std::endl;
		wrapper->tango_ds_class->set_state(Tango::DevState::UNKNOWN);
		close_camera(wrapper);		// Make sure camera is closed.
		return 0;
	}

	int JaiGenicamDisconnectedState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

	int JaiGenicamDisconnectedState::connect(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamDisconnectedState::connect" << std::endl;
		//wrapper->tango_state = wrapper->tango_ds_class->get_state();
		change_state(wrapper, JaiGenicamInitState::Instance());
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

	int JaiGenicamDisconnectedState::close_camera(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamState close camera" << std::endl;
		J_STATUS_TYPE   retval;
		if (wrapper->camera_handle != NULL)
		{
			retval = J_Camera_Close(wrapper->camera_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not close camera! " << wrapper->get_error_string(retval) << std::endl;
				return retval;
			}
			else
			{
				std::cout << "Camera closed." << std::endl;
				wrapper->camera_handle = NULL;
			}			
		}
		return 0;
	};

	int JaiGenicamDisconnectedState::close_factory(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamDisconnectedState close factory" << std::endl;
		J_STATUS_TYPE   retval;
		if (wrapper->factory_handle != NULL)
		{
			retval = J_Factory_Close(wrapper->factory_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not close factory! " << wrapper->get_error_string(retval) << std::endl;
				return retval;
			}
			else
			{
				std::cout << "Factory closed." << std::endl;
				wrapper->factory_handle = NULL;
			}
		}
		return 0;
	};

	/* -----------------------------------------------
		JaiGenicamInitState implementation
	--------------------------------------------------*/
	JaiGenicamInitState* JaiGenicamInitState::_instance = NULL;

	int JaiGenicamInitState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering INIT state" << std::endl;
		wrapper->tango_ds_class->set_state(Tango::DevState::UNKNOWN);
		wrapper->run();
		this->disable_auto_nodes(wrapper);
		return 0;
	}

	int JaiGenicamInitState::run(JaiGenicamConnection* wrapper)
	{
		std::cout << "Attempting connection" << std::endl;
		wrapper->tango_state = wrapper->tango_ds_class->get_state();

		J_STATUS_TYPE   retval;
		bool8_t         has_changed;
		uint32_t        n_cameras;
		int8_t          camera_id_s[J_CAMERA_ID_SIZE];
		uint32_t        size;

		if (wrapper->factory_handle == NULL)
		{
			retval = J_Factory_Open((int8_t*)"" , &wrapper->factory_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout << "Could not open factory!" << wrapper->get_error_string(retval) << std::endl;
				change_state(wrapper, JaiGenicamDisconnectedState::Instance());
				return retval;
			}
		};

		//Update camera list
		retval = J_Factory_UpdateCameraList(wrapper->factory_handle, &has_changed);
		if (retval != J_ST_SUCCESS)
		{
			std::cout << "Could not update camera list!" << wrapper->get_error_string(retval) << std::endl;
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
			return retval;
		}
		else
		{
			std::cout << "Camera list updated." << std::endl;
		}

		// Get the number of Cameras
		retval = J_Factory_GetNumOfCameras(wrapper->factory_handle, &n_cameras);
		if (retval != J_ST_SUCCESS)
		{
			std::cout << "Failure getting number of cameras!" << wrapper->get_error_string(retval) << std::endl;
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
			return retval;
		}
		else
		{
			std::cout << "Found " << n_cameras << " cameras" << std::endl;
		}

		if (n_cameras == 0)
		{
			std::cout << "No cameras were found." << wrapper->get_error_string(retval) << std::endl;
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
			return retval;
		}

		// Get camera ID
		int8_t   s_camera_info[J_CAMERA_INFO_SIZE];
		uint32_t size_ci;
		std::string target_serial;
		target_serial = wrapper->camera_serial;
		bool camera_found = false;

		for (int i=0; i<n_cameras; i++)
		{
			std::cout << "============================================" << std::endl;

			size_ci = (uint32_t)sizeof(s_camera_info);
			size = (uint32_t)sizeof(camera_id_s);
			retval = J_Factory_GetCameraIDByIndex(wrapper->factory_handle, i, camera_id_s, &size);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not get the camera ID!" << wrapper->get_error_string(retval) << std::endl;
				return retval;
			}
			else
			{
				std::cout << "Camera " << i << " ID: " << camera_id_s << std::endl;
			}

			retval = J_Factory_GetCameraInfo(wrapper->factory_handle, camera_id_s, CAM_INFO_SERIALNUMBER, s_camera_info, &size_ci);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not get the camera info!" << wrapper->get_error_string(retval) << std::endl;
				return retval;
			}
			else
			{
				std::cout << "Camera " << i << " serial number: " << s_camera_info << std::endl;
			}
			std::string serial_string;
			for (int k=0; k<size_ci-1; k++)
			{
				serial_string += (char)s_camera_info[k];
			}
			std::cout << "Target serial: " << target_serial << ", this camera " << serial_string << std::endl;
			if (target_serial == serial_string)
			{
				camera_found = true;
				std::cout << "Found camera with serial " << target_serial << ", index " << i << std::endl;
				break;
			}
		}
		if (camera_found == false)
		{
			// Did not find a camera with correct serial number
			wrapper->tango_ds_class->set_status("Camera serial number not found on bus");
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
			return retval;
		}
		else
		{
			retval = J_Camera_Open(wrapper->factory_handle, camera_id_s, &wrapper->camera_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not open camera!" << wrapper->get_error_string(retval) << std::endl;
				change_state(wrapper, JaiGenicamDisconnectedState::Instance());
				return retval;
			}
			else
			{
				std::cout << "Camera open." << std::endl;
			}
		}

		change_state(wrapper, JaiGenicamConnectedState::Instance());	
		return 0;
	};

	int JaiGenicamInitState::disable_auto_nodes(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamInitState::disable_auto_nodes" << std::endl;
		
		J_STATUS_TYPE		retval;
		J_NODE_TYPE			node_type;
		std::stringstream	err_msg;
		int64_t				int_value;
		uint32_t			n_nodes;
		string				node_name;		
		string				auto_name = "auto";
		char				char_buffer_p[512];
		uint32_t			char_buffer_size;
		NODE_HANDLE			h_node;
		std::size_t			found_pos;
		vector<string>		off_names;
		off_names.push_back("exposure");
		off_names.push_back("gain");

		retval = J_Camera_GetNumOfNodes(wrapper->camera_handle, &n_nodes);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting number of nodes, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		std::cout << "Found " << n_nodes << " nodes." << std::endl;

		for (int i_node=0; i_node < n_nodes; i_node++)
		{
			retval = J_Camera_GetNodeByIndex(wrapper->camera_handle, i_node, &h_node);
			if (retval != J_ST_SUCCESS)
			{ 
				err_msg <<  "Failure getting node index " << i_node << ", returned " << wrapper->get_error_string(retval) ;
				std::cout << err_msg.str() << std::endl;
				return retval;
			}
			char_buffer_size = 512;
			retval = J_Node_GetName(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
			if (retval != J_ST_SUCCESS)
			{ 
				err_msg <<  "Failure getting node name for index " << i_node << ", returned " << wrapper->get_error_string(retval) ;
				std::cout << err_msg.str() << std::endl;
				return retval;
			}
			node_name = char_buffer_p;
			std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
			found_pos = node_name.find(auto_name);
			if (found_pos != node_name.npos)
			{
				std::cout << "Found name with auto: " << node_name << "." << std::endl;
				for (auto & off_name : off_names)
				{
					found_pos = node_name.find(off_name);
					if (found_pos != node_name.npos)
					{						
						J_Node_GetType(h_node, &node_type);
						if (node_type == J_IEnumeration)
						{
							std::cout << "Turning off." << std::endl;
							int_value = 0;
							retval = J_Node_SetValueInt64(h_node, true, int_value);
						}
					}
				}
			}
		}

	};



	/* -----------------------------------------------
		JaiGenicamConnectedState implementation
	--------------------------------------------------*/
	JaiGenicamConnectedState* JaiGenicamConnectedState::_instance = NULL;

	int JaiGenicamConnectedState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering CONNECTED state" << std::endl;
		wrapper->tango_ds_class->set_state(Tango::DevState::ON);

		int retval;
//		retval = this->stop_capture(wrapper);

		std::thread t = std::thread(&JaiGenicamConnectedState::stop_capture, this, wrapper);
		t.detach();

		return 0;

	}

	int JaiGenicamConnectedState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};
	
	int JaiGenicamConnectedState::start_capture(JaiGenicamConnection* wrapper)
	{		
		change_state(wrapper, JaiGenicamRunningState::Instance());
		return 0;
	};

	int JaiGenicamConnectedState::stop_capture(JaiGenicamConnection* wrapper)
	{
		std::cout << "Stopping aquisition" << std::endl;

		J_STATUS_TYPE retval;
		std::stringstream err_msg;

		// Lock the mutex to keep start_capture to execute while we are still stopping
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);	
			// Stop Acquisition
			if (wrapper->camera_handle != NULL) 
			{
				retval = J_Camera_ExecuteCommand(wrapper->camera_handle, NODE_NAME_ACQSTOP);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not Stop Acquisition! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
			}

			if(wrapper->capture_thread_handle != NULL)
			{
				// Close stream
				retval = J_Image_CloseStream(wrapper->capture_thread_handle);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not close Stream!! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				wrapper->capture_thread_handle = NULL;
			}
		lock.unlock();
		return 0;
	};


	template<typename T>
	int JaiGenicamConnectedState::get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node)
	{
		std::cout << "JaiGenicamConnectedState::get_node for " << node->name << " of type " << node->type << std::endl;
		
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;
		char				char_buffer_p[512];
		uint32_t			char_buffer_size;
		NODE_HANDLE			h_node;


		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)node->name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << node->name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		
		char_buffer_size = 512;
		retval = J_Node_GetDescription(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node description, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			
		}
		else
		{
			node->description = char_buffer_p;
		}

		char_buffer_size = 512;
		retval = J_Node_GetUnit(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node unit, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;			
		}
		else
		{
			node->unit = char_buffer_p;
		}

		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
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
				// Get actual value:
				retval = J_Node_GetValueInt64(h_node, true, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->value = int_value;
//				std::cout << "Current value: " << int_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int min value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->min_value = int_value;
				// Get max value:
				retval = J_Node_GetMaxInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int max value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->max_value = int_value;
				node->valid = true;
				break;
			}

			case J_IFloat:
			case J_ISwissKnife:
			{
				// It was a float value
//				std::cout << "Node type float" << std::endl;

				// Get actual value:
				retval = J_Node_GetValueDouble(h_node, true, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->value = double_value;
//				std::cout << "Current value: " << double_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double min value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->min_value = double_value;
				// Get max value:
				retval = J_Node_GetMaxDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double max value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->max_value = double_value;			
				node->valid = true;
				break;
			}

			case J_IEnumeration:
			case J_IEnumEntry:
				{
					// It was an integer value
//					std::cout << "Node type enum" << std::endl;
					// Get actual value:
					retval = J_Node_GetValueInt64(h_node, true, &int_value);				
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting enum value, returned " << wrapper->get_error_string(retval) ;
						std::cout << err_msg.str() << std::endl;
						return retval;
					}
					node->value = int_value;
					std::cout << "Current value: " << int_value << std::endl;
					
					uint32_t num_enum = 0;
					retval = J_Node_GetNumOfEnumEntries(h_node, &num_enum);
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting number of enum entries, returned " << wrapper->get_error_string(retval) ;
						std::cout << err_msg.str() << std::endl;						
					}
					std::cout << "Number of enum entries: " << num_enum << std::endl;

					NODE_HANDLE enum_entry;
					J_NODE_ACCESSMODE access_mode;
					std::stringstream	desc_stream;
					for (uint32_t j=0; j<num_enum; j++)
					{
						retval = J_Node_GetEnumEntryByIndex(h_node, j, &enum_entry);
						if (retval != J_ST_SUCCESS)
						{
							err_msg <<  "Failure getting enum entry " << j << ", returned " << wrapper->get_error_string(retval) ;
							std::cout << err_msg.str() << std::endl;						
						}
						else
						{
							char_buffer_size = 512;
							retval = J_Node_GetName(enum_entry, (int8_t*)char_buffer_p, &char_buffer_size);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node name, returned " << wrapper->get_error_string(retval) ;
								std::cout << err_msg.str() << std::endl;
			
							}
							else
							{
								std::cout << "Enum entry " << j << ": " << char_buffer_p << std::endl;
							}
							retval = J_Node_GetAccessMode(enum_entry, &access_mode);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node access mode, returned " << wrapper->get_error_string(retval) ;
								std::cout << err_msg.str() << std::endl;
			
							}
							else
							{								
								switch (access_mode)
								{
								case _J_NODE_ACCESSMODE_TYPE::RO:
								case _J_NODE_ACCESSMODE_TYPE::RW:
								case _J_NODE_ACCESSMODE_TYPE::WO:
									desc_stream << "Enum value " << j << ": " << char_buffer_p << std::endl;
									
								}

								
							}
						} // For enum entries
						node->description = desc_stream.str();

					}

					
					node->valid = true;
					break;
				}

			case J_IStringReg:			
			default:
				node->valid = false;
		}; // switch (node_type)
		std::cout << "Node description: " << node->description << std::endl;
		return 0;
	}; // JaiGenicamConnectedState::get_node

	int JaiGenicamConnectedState::get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;

		std::cout << "get_node_value for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		// Check if the node type is compatible and select the correct get_ function
		if (node_type == J_IFloat)
		{
			retval = J_Node_GetValueDouble(h_node, true, value_p);
		}
		else if (node_type == J_IInteger)
		{
			retval = J_Node_GetValueInt64(h_node, true, &int_value);
			*value_p = (double)int_value;
		}
		else
		{
			err_msg <<  "Wrong node type, should be " << J_IFloat << " (double) or " << J_IInteger << " (integer), is " << node_type ;
			std::cout << err_msg.str() << std::endl;
			return -1;
		}
		
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	} // JaiGenicamConnectedState::get_node_value

	int JaiGenicamConnectedState::get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		std::cout << "get_node_value integer for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		if (node_type != J_IInteger)
		{
			if (node_type != J_IEnumeration)
			{
				err_msg <<  "Wrong type, should be " << J_IInteger << " (integer) or " << J_IEnumeration << ", is " << node_type ;
				std::cout << err_msg.str() << std::endl;
				return -1;
			}
		}
		retval = J_Node_GetValueInt64(h_node, true, value_p);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	} // JaiGenicamConnectedState::get_node_value

	int JaiGenicamConnectedState::set_node_value(JaiGenicamConnection* wrapper, std::string name, double value)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;

		std::cout << "set_node_value for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		// Check if the node type is compatible and select the correct get_ function
		if (node_type == J_IFloat)
		{
			retval = J_Node_SetValueDouble(h_node, true, value);
		}
		else if (node_type == J_IInteger)
		{
			int_value = (int64_t)(value + 0.5);
			retval = J_Node_SetValueInt64(h_node, true, int_value);
		}
		else if (node_type == J_IEnumeration)
		{
			int_value = (int64_t)(value + 0.5);
			retval = J_Node_SetValueInt64(h_node, true, int_value);
		}
		else
		{
			err_msg <<  "Wrong node type, should be " << J_IFloat << " (double), " << J_IInteger << " (integer), or "<< J_IEnumeration << " (enumeration) - this is " << node_type ;
			std::cout << err_msg.str() << std::endl;
			return -1;
		}
		
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	}

	int JaiGenicamConnectedState::get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		switch (node_type)
		{
		case _J_NODE_TYPE_TYPE::J_IFloat:
			(*type) = "double";
			return 0;
		case _J_NODE_TYPE_TYPE::J_IInteger:
			(*type) = "integer";
			return 0;
		case _J_NODE_TYPE_TYPE::J_IEnumeration:
			(*type) = "enumeration";
			return 0;
		default:
			(*type) = "unknown";
			return 0;
		}
		return -1;
	}

	int JaiGenicamConnectedState::get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p)
	{
//		std::cout << "JaiGenicamConnectedState::get_node_info for " << name << std::endl;
		generic_node_p->name = name;
		
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;
		char				char_buffer_p[512];
		uint32_t			char_buffer_size;
		NODE_HANDLE			h_node;


		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			return retval;
		}
		
		char_buffer_size = 512;
		retval = J_Node_GetDescription(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node description, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			generic_node_p->description = "";
			
		}
		else
		{
			generic_node_p->description = char_buffer_p;
		}

		char_buffer_size = 512;
		retval = J_Node_GetUnit(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node unit, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();	
			generic_node_p->unit = "";
		}
		else
		{
			generic_node_p->unit = char_buffer_p;
		}

		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
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
				generic_node_p->type = J_NODE_TYPE::J_IInteger;
				// Get actual value:
				retval = J_Node_GetValueInt64(h_node, true, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->value_i = int_value;
//				std::cout << "Current value: " << int_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int min value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->min_value_i = int_value;
				generic_node_p->min_value_d = (double)int_value;
				// Get max value:
				retval = J_Node_GetMaxInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int max value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->max_value_i = int_value;
				generic_node_p->max_value_d = (double)int_value;
				generic_node_p->valid = true;
				break;
			}

			case J_IFloat:
			case J_ISwissKnife:
			{
				// It was a float value
				generic_node_p->type = J_NODE_TYPE::J_IFloat;
//				std::cout << "Node type float" << std::endl;

				// Get actual value:
				retval = J_Node_GetValueDouble(h_node, true, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->value_d = double_value;
//				std::cout << "Current value: " << double_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double min value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->min_value_d = double_value;
				generic_node_p->min_value_i = (int64_t)double_value;
				// Get max value:
				retval = J_Node_GetMaxDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double max value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->max_value_d = double_value;
				generic_node_p->max_value_i = (int64_t)double_value;
				generic_node_p->valid = true;
				break;
			}

			case J_IEnumeration:
			case J_IEnumEntry:
				{
					// It was an enumeration value
					generic_node_p->type = J_NODE_TYPE::J_IEnumeration;
//					std::cout << "Node type enum" << std::endl;
					// Get actual value:
					retval = J_Node_GetValueInt64(h_node, true, &int_value);				
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting enum value, returned " << wrapper->get_error_string(retval) << std::endl;
						std::cout << err_msg.str();
						return retval;
					}
					generic_node_p->value_i = int_value;
//					std::cout << "Current value: " << int_value << std::endl;
					
					uint32_t num_enum = 0;
					retval = J_Node_GetNumOfEnumEntries(h_node, &num_enum);
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting number of enum entries, returned " << wrapper->get_error_string(retval) << std::endl;
						
					}
//					std::cout << "Number of enum entries: " << num_enum << std::endl;

					// Building the list of available enum values
					NODE_HANDLE enum_entry;
					J_NODE_ACCESSMODE access_mode;
					std::stringstream	desc_stream;		// Used to generate the description string
					std::string entry_map_string;
					desc_stream << generic_node_p->description << std::endl;
					desc_stream << "Valid entries: ";
					generic_node_p->enum_names.clear();
					for (uint32_t j=0; j<num_enum; j++)
					{
						retval = J_Node_GetEnumEntryByIndex(h_node, j, &enum_entry);
						if (retval != J_ST_SUCCESS)
						{
							err_msg <<  "Failure getting enum entry " << j << ", returned " << wrapper->get_error_string(retval) << std::endl;
							std::cout << err_msg.str();						
						}
						else
						{
							char_buffer_size = 512;
							retval = J_Node_GetName(enum_entry, (int8_t*)char_buffer_p, &char_buffer_size);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node name, returned " << wrapper->get_error_string(retval) << std::endl;
								std::cout << err_msg.str();
			
							}
							else
							{
								std::cout << "Enum entry " << j << ": " << char_buffer_p << std::endl;
							}
							retval = J_Node_GetAccessMode(enum_entry, &access_mode);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node access mode, returned " << wrapper->get_error_string(retval) << std::endl;
								std::cout << err_msg.str();
			
							}
							else
							{								
								switch (access_mode)
								{
									// Only add enum entries that are implemented and available
								case _J_NODE_ACCESSMODE_TYPE::RO:
								case _J_NODE_ACCESSMODE_TYPE::RW:
								case _J_NODE_ACCESSMODE_TYPE::WO:
									retval = J_Node_GetEnumEntryValue(enum_entry, &int_value);
									entry_map_string = char_buffer_p+9+2+generic_node_p->name.length();
//									desc_stream << "Enum index " << j << ": " << char_buffer_p << ", value " << int_value << std::endl;
									desc_stream << char_buffer_p+9+2+generic_node_p->name.length() << ", ";
//									std::cout << "Cut enum name: " << entry_map_string << std::endl;
									generic_node_p->enum_names.push_back(char_buffer_p+9+2+generic_node_p->name.length());
									generic_node_p->enum_entry_map[entry_map_string] = int_value;
									generic_node_p->enum_value_map[int_value] = char_buffer_p+9+2+generic_node_p->name.length();
								}

								
							}
						} // For enum entries
						generic_node_p->description = desc_stream.str();
						
					}

					
					generic_node_p->valid = true;
					break;
				}

			case J_IStringReg:			
			default:
				generic_node_p->valid = false;
		}; // switch (node_type)
//		std::cout << "Node description: " << generic_node_p->description << std::endl;
		return 0;
	}; // JaiGenicamConnectedState::get_node_info
	


	/* -----------------------------------------------
		JaiGenicamRunningState implementation
	--------------------------------------------------*/
	JaiGenicamRunningState* JaiGenicamRunningState::_instance = NULL;

	int JaiGenicamRunningState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering RUNNING state" << std::endl;
		wrapper->tango_ds_class->set_state(Tango::DevState::RUNNING);
		int retval;
		retval = this->start_capture(wrapper);

		return 0;
	};

	int JaiGenicamRunningState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

	int JaiGenicamRunningState::connect(JaiGenicamConnection* wrapper)
	{
		change_state(wrapper, JaiGenicamConnectedState::Instance());
		return 0;
	};

	int JaiGenicamRunningState::start_capture(JaiGenicamConnection* wrapper)
	{
		// Spawn thread starting capture to preserve responsiveness
		std::thread t(&JaiGenicamRunningState::execute_start_capture, this, wrapper);
		t.detach();
		return 0;
	}

	int JaiGenicamRunningState::execute_start_capture(JaiGenicamConnection* wrapper)
	{
		std::cout << "Starting aquisition" << std::endl;

		J_STATUS_TYPE   retval;
		std::stringstream err_msg;

		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);	
			// Close old stream if active
			if(wrapper->capture_thread_handle != NULL)
			{				
				// Stop acquisition
				retval = J_Camera_ExecuteCommand(wrapper->camera_handle, NODE_NAME_ACQSTOP);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not stop aquisition! " << wrapper->get_error_string(retval);
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				// Close stream
				retval = J_Image_CloseStream(wrapper->capture_thread_handle);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not close Stream!! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				wrapper->capture_thread_handle = NULL;				
			}

			// Get the pixelformat from the camera
			uint64_t jaiPixelFormat = 0;
			retval = this->get_node(wrapper, &wrapper->pixelformat_node);

			retval = J_Image_Get_PixelFormat(wrapper->camera_handle, wrapper->pixelformat_node.value, &jaiPixelFormat);
			if (retval != J_ST_SUCCESS) {
				err_msg << "Could not get pixelformat! " << wrapper->get_error_string(retval);
				std::cout << err_msg.str() << std::endl;
				return retval;
			}

			// Calculate number of bits (not bytes) per pixel using macro
			int bpp = J_BitsPerPixel(jaiPixelFormat);
			retval = this->get_node(wrapper, &wrapper->imagewidth_node);
			int64_t image_width = wrapper->imagewidth_node.value;
			retval = this->get_node(wrapper, &wrapper->imageheight_node);
			int64_t image_height = wrapper->imageheight_node.value;

			std::cout << "Image parameters: " << std::endl << "  height..." << image_height << std::endl << "   width..." << image_width
				<< std::endl << "     bpp..." << bpp << std::endl << "    size..." << (image_height*image_width*bpp)/8 << std::endl;

			// Open stream
			retval = J_Image_OpenStream(wrapper->camera_handle, 0, 
				reinterpret_cast<J_IMG_CALLBACK_OBJECT>(wrapper), 
				reinterpret_cast<J_IMG_CALLBACK_FUNCTION>(&JaiGenicamConnection::capture_stream_callback), 
				&wrapper->capture_thread_handle, (image_height*image_width*bpp)/8);
			if (retval != J_ST_SUCCESS) {
				err_msg << "Could not open stream! " << wrapper->get_error_string(retval);
				std::cout << err_msg.str() << std::endl;
				return retval;
			}

			// Start Acquisition
			retval = J_Camera_ExecuteCommand(wrapper->camera_handle, NODE_NAME_ACQSTART);
			if (retval != J_ST_SUCCESS)
			{
				err_msg << "Could not start aquisition! " << wrapper->get_error_string(retval);
				std::cout << err_msg.str() << std::endl;
				return retval;
			}
		lock.unlock();
		
		return 0;
	};

	int JaiGenicamRunningState::stop_capture(JaiGenicamConnection* wrapper)
	{
		change_state(wrapper, JaiGenicamConnectedState::Instance());
		return 0;
	};

	int JaiGenicamRunningState::close_camera(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamRunningState close camera" << std::endl;
		J_STATUS_TYPE   retval;
		std::stringstream err_msg;

		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);	
			// Stop Acquisition
			if (wrapper->camera_handle != NULL) 
			{
				retval = J_Camera_ExecuteCommand(wrapper->camera_handle, NODE_NAME_ACQSTOP);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not Stop Acquisition! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
			}

			if(wrapper->capture_thread_handle != NULL)
			{
				// Close stream
				retval = J_Image_CloseStream(wrapper->capture_thread_handle);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not close Stream!! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				wrapper->capture_thread_handle = NULL;
			}
		lock.unlock();

		if (wrapper->camera_handle != NULL)
		{
			retval = J_Camera_Close(wrapper->camera_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not close camera! " << wrapper->get_error_string(retval) << std::endl;
			}
			else
			{
				std::cout << "Camera closed." << std::endl;
				wrapper->camera_handle = NULL;
			}
			std::cout << "Changing state to disconnected." << std::endl;
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
		}
		return 0;
	};


	template<typename T>
	int JaiGenicamRunningState::get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node)
	{
		std::cout << "JaiGenicamConnectedState::get_node for " << node->name << " of type " << node->type << std::endl;
		
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;
		NODE_HANDLE			h_node;

		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)node->name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << node->name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
//			throw std::invalid_argument(err_msg.str().c_str());
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		std::cout << "Node type " << (int)node_type << std::endl;
		switch (node_type)
		{
			case J_IInteger:
			case J_IRegister:
			case J_IBoolean:
			case J_IIntSwissKnife:
			case J_IIntReg:
			{
				// It was an integer value
				std::cout << "Node type integer" << std::endl;
				// Get actual value:
				retval = J_Node_GetValueInt64(h_node, true, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->value = int_value;
				std::cout << "Current value: " << int_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int min value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->min_value = int_value;
				// Get max value:
				retval = J_Node_GetMaxInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int max value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->max_value = int_value;
				node->valid = true;
				break;
			}

			case J_IFloat:
			case J_ISwissKnife:
			{
				// It was a float value
				std::cout << "Node type float" << std::endl;

				// Get actual value:
				retval = J_Node_GetValueDouble(h_node, true, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->value = double_value;
				std::cout << "Current value: " << double_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double min value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->min_value = double_value;
				// Get max value:
				retval = J_Node_GetMaxDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double max value, returned " << wrapper->get_error_string(retval) ;
					std::cout << err_msg.str() << std::endl;
					return retval;
				}
				node->max_value = double_value;			
				node->valid = true;
				break;
			}

			case J_IEnumeration:
			case J_IEnumEntry:
				{
					// It was an integer value
					std::cout << "Node type enum" << std::endl;
					// Get actual value:
					retval = J_Node_GetValueInt64(h_node, true, &int_value);				
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting enum value, returned " << wrapper->get_error_string(retval) ;
						std::cout << err_msg.str() << std::endl;
						return retval;
					}
					node->value = int_value;
					std::cout << "Current value: " << int_value << std::endl;
					
					node->valid = true;
					break;
				}

			case J_IStringReg:			
			default:
				node->valid = false;
		}; // switch (node_type)
		return 0;
	}; // JaiGenicamRunningState::get_node

	int JaiGenicamRunningState::get_node_value(JaiGenicamConnection* wrapper, std::string name, double* value_p)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;

//		std::cout << "get_node_value for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		// Check if the node type is compatible and select the correct get_ function
		if (node_type == J_IFloat)
		{
			retval = J_Node_GetValueDouble(h_node, true, value_p);
		}
		else if (node_type == J_IInteger)
		{
			retval = J_Node_GetValueInt64(h_node, true, &int_value);
			*value_p = (double)int_value;
		}
		else
		{
			err_msg <<  "Wrong node type, should be " << J_IFloat << " (double) or " << J_IInteger << " (integer), is " << node_type ;
			std::cout << err_msg.str() << std::endl;
			return -1;
		}
		
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	} // // JaiGenicamRunningState::get_node_value	

	int JaiGenicamRunningState::get_node_value(JaiGenicamConnection* wrapper, std::string name, int64_t* value_p)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

//		std::cout << "get_node_value integer for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		if (node_type != J_IInteger)
		{
			if (node_type != J_IEnumeration)
			{
				err_msg <<  "Wrong type, should be " << J_IInteger << " (integer) or " << J_IEnumeration << ", is " << node_type ;
				std::cout << err_msg.str() << std::endl;
				return -1;
			}
		}
		retval = J_Node_GetValueInt64(h_node, true, value_p);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	} // JaiGenicamRunningState::get_node_value int

	int JaiGenicamRunningState::set_node_value(JaiGenicamConnection* wrapper, std::string name, double value)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;

		std::cout << "set_node_value for node named " << name.c_str() << std::endl;
		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		// Check if the node type is compatible and select the correct get_ function
		if (node_type == J_IFloat)
		{
			retval = J_Node_SetValueDouble(h_node, true, value);
		}
		else if (node_type == J_IInteger)
		{
			int_value = (int64_t)(value + 0.5);
			retval = J_Node_SetValueInt64(h_node, true, int_value);
		}
		else if (node_type == J_IEnumeration)
		{
			int_value = (int64_t)(value + 0.5);
			retval = J_Node_SetValueInt64(h_node, true, int_value);
		}
		else
		{
			err_msg <<  "Wrong node type, should be " << J_IFloat << " (double), " << J_IInteger << " (integer), or "<< J_IEnumeration << " (enumeration) - this is " << node_type ;
			std::cout << err_msg.str() << std::endl;
			return -1;
		}
		
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node value, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		return 0;
	} //JaiGenicamRunningState::set_node_value

	int JaiGenicamRunningState::get_node_type(JaiGenicamConnection* wrapper, std::string name, std::string* type)
	{
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		NODE_HANDLE			h_node;

		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) ;
			std::cout << err_msg.str() << std::endl;
			return retval;
		}
		switch (node_type)
		{
		case _J_NODE_TYPE_TYPE::J_IFloat:
			(*type) = "double";
			return 0;
		case _J_NODE_TYPE_TYPE::J_IInteger:
			(*type) = "integer";
			return 0;
		case _J_NODE_TYPE_TYPE::J_IEnumeration:
			(*type) = "enumeration";
			return 0;
		default:
			(*type) = "unknown";
			return 0;
		}
		return -1;
	} // JaiGenicamRunningState::get_node_type

	int JaiGenicamRunningState::get_node_info(JaiGenicamConnection* wrapper, std::string name, GenicamGenericNode* generic_node_p)
	{
//		std::cout << "JaiGenicamConnectedState::get_node_info for " << name << std::endl;
		generic_node_p->name = name;
		
		J_NODE_TYPE			node_type;
		J_STATUS_TYPE		retval;
		std::stringstream	err_msg;
		int64_t				int_value;
		uint32_t			uint_value;
		string				string_value;		
		double				double_value;
		char				char_buffer_p[512];
		uint32_t			char_buffer_size;
		NODE_HANDLE			h_node;


		retval = J_Camera_GetNodeByName(wrapper->camera_handle, (int8_t*)name.c_str(), &h_node);
		if (retval != J_ST_SUCCESS)
		{ 
			err_msg <<  "Failure getting node " << name << ", returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			return retval;
		}
		
		char_buffer_size = 512;
		retval = J_Node_GetDescription(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node description, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
			generic_node_p->description = "";
			
		}
		else
		{
			generic_node_p->description = char_buffer_p;
		}

		char_buffer_size = 512;
		retval = J_Node_GetUnit(h_node, (int8_t*)char_buffer_p, &char_buffer_size);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting node unit, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();	
			generic_node_p->unit = "";
		}
		else
		{
			generic_node_p->unit = char_buffer_p;
		}

		retval = J_Node_GetType(h_node, &node_type);
		if (retval != J_ST_SUCCESS)
		{
			err_msg <<  "Failure getting type, returned " << wrapper->get_error_string(retval) << std::endl;
			std::cout << err_msg.str();
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
				generic_node_p->type = J_NODE_TYPE::J_IInteger;
				// Get actual value:
				retval = J_Node_GetValueInt64(h_node, true, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->value_i = int_value;
//				std::cout << "Current value: " << int_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int min value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->min_value_i = int_value;
				generic_node_p->min_value_d = (double)int_value;
				// Get max value:
				retval = J_Node_GetMaxInt64(h_node, &int_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting int max value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->max_value_i = int_value;
				generic_node_p->max_value_d = (double)int_value;
				generic_node_p->valid = true;
				break;
			}

			case J_IFloat:
			case J_ISwissKnife:
			{
				// It was a float value
				generic_node_p->type = J_NODE_TYPE::J_IFloat;
//				std::cout << "Node type float" << std::endl;

				// Get actual value:
				retval = J_Node_GetValueDouble(h_node, true, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->value_d = double_value;
//				std::cout << "Current value: " << double_value << std::endl;
				// Get min value:
				retval = J_Node_GetMinDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double min value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->min_value_d = double_value;
				generic_node_p->min_value_i = (int64_t)double_value;
				// Get max value:
				retval = J_Node_GetMaxDouble(h_node, &double_value);
				if (retval != J_ST_SUCCESS)
				{
					err_msg <<  "Failure getting double max value, returned " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				generic_node_p->max_value_d = double_value;
				generic_node_p->max_value_i = (int64_t)double_value;
				generic_node_p->valid = true;
				break;
			}

			case J_IEnumeration:
			case J_IEnumEntry:
				{
					// It was an enumeration value
					generic_node_p->type = J_NODE_TYPE::J_IEnumeration;
//					std::cout << "Node type enum" << std::endl;
					// Get actual value:
					retval = J_Node_GetValueInt64(h_node, true, &int_value);				
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting enum value, returned " << wrapper->get_error_string(retval) << std::endl;
						std::cout << err_msg.str();
						return retval;
					}
					generic_node_p->value_i = int_value;
//					std::cout << "Current value: " << int_value << std::endl;
					
					uint32_t num_enum = 0;
					retval = J_Node_GetNumOfEnumEntries(h_node, &num_enum);
					if (retval != J_ST_SUCCESS)
					{
						err_msg <<  "Failure getting number of enum entries, returned " << wrapper->get_error_string(retval) << std::endl;
						
					}
//					std::cout << "Number of enum entries: " << num_enum << std::endl;

					// Building the list of available enum values
					NODE_HANDLE enum_entry;
					J_NODE_ACCESSMODE access_mode;
					std::stringstream	desc_stream;		// Used to generate the description string
					std::string entry_map_string;
					desc_stream << generic_node_p->description << std::endl;
					desc_stream << "Valid entries: ";
					generic_node_p->enum_names.clear();
					for (uint32_t j=0; j<num_enum; j++)
					{
						retval = J_Node_GetEnumEntryByIndex(h_node, j, &enum_entry);
						if (retval != J_ST_SUCCESS)
						{
							err_msg <<  "Failure getting enum entry " << j << ", returned " << wrapper->get_error_string(retval) << std::endl;
							std::cout << err_msg.str();						
						}
						else
						{
							char_buffer_size = 512;
							retval = J_Node_GetName(enum_entry, (int8_t*)char_buffer_p, &char_buffer_size);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node name, returned " << wrapper->get_error_string(retval) << std::endl;
								std::cout << err_msg.str();
			
							}
							else
							{
								std::cout << "Enum entry " << j << ": " << char_buffer_p << std::endl;
							}
							retval = J_Node_GetAccessMode(enum_entry, &access_mode);
							if (retval != J_ST_SUCCESS)
							{
								err_msg <<  "Failure getting enum entry node access mode, returned " << wrapper->get_error_string(retval) << std::endl;
								std::cout << err_msg.str();
			
							}
							else
							{								
								switch (access_mode)
								{
									// Only add enum entries that are implemented and available
								case _J_NODE_ACCESSMODE_TYPE::RO:
								case _J_NODE_ACCESSMODE_TYPE::RW:
								case _J_NODE_ACCESSMODE_TYPE::WO:
									retval = J_Node_GetEnumEntryValue(enum_entry, &int_value);
									entry_map_string = char_buffer_p+9+2+generic_node_p->name.length();
//									desc_stream << "Enum index " << j << ": " << char_buffer_p << ", value " << int_value << std::endl;
									desc_stream << char_buffer_p+9+2+generic_node_p->name.length() << ", ";
//									std::cout << "Cut enum name: " << entry_map_string << std::endl;
									generic_node_p->enum_names.push_back(char_buffer_p+9+2+generic_node_p->name.length());
									generic_node_p->enum_entry_map[entry_map_string] = int_value;
									generic_node_p->enum_value_map[int_value] = char_buffer_p+9+2+generic_node_p->name.length();
								}

								
							}
						} // For enum entries
						generic_node_p->description = desc_stream.str();
						
					}

					
					generic_node_p->valid = true;
					break;
				}

			case J_IStringReg:			
			default:
				generic_node_p->valid = false;
		}; // switch (node_type)
//		std::cout << "Node description: " << generic_node_p->description << std::endl;
		return 0;
	}; // JaiGenicamRunningState::get_node_info

	/* -----------------------------------------------
		JaiGenicamFaultState implementation
	--------------------------------------------------*/
	JaiGenicamFaultState* JaiGenicamFaultState::_instance = NULL;

	int JaiGenicamFaultState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering FAULT state" << std::endl;

		int result;

		// Stop capture if running
		result = this->stop_capture(wrapper);
		// Next, try closing the camera:
		result = this->close_camera(wrapper);
		if (result != 0)
		{
			// Could not close camera, so go to disconnected state
			change_state(wrapper, JaiGenicamDisconnectedState::Instance());
		}
		// Reconnect to camera:
		change_state(wrapper, JaiGenicamInitState::Instance());
		return 0;
	};

	int JaiGenicamFaultState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

	int JaiGenicamFaultState::stop_capture(JaiGenicamConnection* wrapper)
	{
		std::cout << "Stopping aquisition" << std::endl;

		J_STATUS_TYPE retval;
		std::stringstream err_msg;

		// Lock the mutex to keep start_capture to execute while we are still stopping
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);	
			// Stop Acquisition
			if (wrapper->camera_handle != NULL) 
			{
				retval = J_Camera_ExecuteCommand(wrapper->camera_handle, NODE_NAME_ACQSTOP);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not Stop Acquisition! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
			}

			if(wrapper->capture_thread_handle != NULL)
			{
				// Close stream
				retval = J_Image_CloseStream(wrapper->capture_thread_handle);
				if (retval != J_ST_SUCCESS)
				{
					err_msg << "Could not close Stream!! " << wrapper->get_error_string(retval) << std::endl;
					std::cout << err_msg.str();
					return retval;
				}
				wrapper->capture_thread_handle = NULL;
			}
		lock.unlock();
		return 0;
	};

	int JaiGenicamFaultState::close_camera(JaiGenicamConnection* wrapper)
	{
		std::cout << "JaiGenicamFaultState close camera" << std::endl;
		J_STATUS_TYPE   retval;
		if (wrapper->camera_handle != NULL)
		{
			retval = J_Camera_Close(wrapper->camera_handle);
			if (retval != J_ST_SUCCESS)
			{
				std::cout <<  "Could not close camera! " << wrapper->get_error_string(retval) << std::endl;
				return retval;
			}
			else
			{
				std::cout << "Camera closed." << std::endl;
				wrapper->camera_handle = NULL;
			}			
		}
		return 0;
	};

} // namespace JaiGenicamConnection_ns