#include "JaiGenicamConnection.h"
#include <chrono>
#include "JaiGenicamDS.h"
#include <iostream>
#include <stdexcept>
#include <math.h>

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
		
		GenicamNode<uint64_t> gn = { "GainRaw",	// name
									 "int",		// type
									 0,			// value
									 0,			// min_value
									 0,			// max_value
									 false };	// valid
		this->gain_node = gn;
		GenicamNode<double> en = { "ExposureTime", "double", 0, 0, 0, false };
		this->exposuretime_node = en;
		GenicamNode<uint64_t> fn = { "JAIAcquisitionFrameRate", "enum", 0, 0, 0, false };
		this->framerate_node = fn;
		GenicamNode<uint64_t> wn = { "Width", "int", 0, 0, 0, false };
		this->imagewidth_node = wn;
		GenicamNode<uint64_t> hn = { "Height", "int", 0, 0, 0, false };
		this->imageheight_node = hn;
		GenicamNode<uint64_t> pfn = { "PixelFormat",	// name
									 "enum",		// type
									 0,			// value
									 0,			// min_value
									 0,			// max_value
									 false };	// valid
		this->pixelformat_node = pfn;


		this->jai_genicam_state = JaiGenicamDisconnectedState::Instance();
		std::cout << "State instance: " << this->jai_genicam_state << std::endl;
	}


	JaiGenicamConnection::~JaiGenicamConnection(void)
	{
		std::cout << "JaiGenicamConnection destructor" << std::endl;
		close_camera();
		close_factory();
		delete this->jai_genicam_state;
	}

	void JaiGenicamConnection::change_state(JaiGenicamState* new_state)
	{
		this->jai_genicam_state = new_state;
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
		std::cout << "GenicamConnection::get_gain" << std::endl; 
		return this->jai_genicam_state->get_gain(this, value_p);
	};

	int JaiGenicamConnection::get_image(uint32_t* width_p, uint32_t* height_p, uint16_t* image_p)
	{
		std::cout << "GenicamConnection::get_image" << std::endl; 
		return this->jai_genicam_state->get_image(this, width_p, height_p, image_p);
	};

	int JaiGenicamConnection::get_image_info(J_tIMAGE_INFO* aq_image_info_p)
	{
		std::cout << "GenicamConnection::get_image_info" << std::endl; 
		return this->jai_genicam_state->get_image_info(this, aq_image_info_p);
	}


	template<typename T>
	int JaiGenicamConnection::get_node(GenicamNode<T>* node)
	{
		std::cout << "GenicamConnection::get_node" << std::endl;
		return this->jai_genicam_state->get_node(this, node);
	}
	
	int JaiGenicamConnection::get_node_value(std::string name, double* value_p)
	{
		std::cout << "GenicamConnection::get_node_value" << std::endl;
		return this->jai_genicam_state->get_node_value(this, name, value_p);
	}

	int JaiGenicamConnection::get_node_value(std::string name, int64_t* value_p)
	{
		std::cout << "GenicamConnection::get_node_value" << std::endl;
		return this->jai_genicam_state->get_node_value(this, name, value_p);
	}
			
	int JaiGenicamConnection::set_node_value(std::string name, double value)
	{
		std::cout << "GenicamConnection::set_node_value" << std::endl;
		return this->jai_genicam_state->set_node_value(this, name, value);

	}


	void __stdcall JaiGenicamConnection::capture_stream_callback(J_tIMAGE_INFO * aq_imageinfo_p)
	{
		J_STATUS_TYPE   retval;


		std::unique_lock<std::mutex> lock(this->camera_mtx);
			// The state of the buffer is confirmed.

			if(this->image_buffer.pImageBuffer != NULL)
			{
				// Already when the buffer exists, and the size is different:
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
			// J_Image_FromRawToImage(aq_imageinfo_p, &this->image_buffer);
			std::copy(aq_imageinfo_p->pImageBuffer, 
					aq_imageinfo_p->pImageBuffer + aq_imageinfo_p->iSizeX*aq_imageinfo_p->iSizeY,
					this->image_buffer.pImageBuffer);
			// this->image_buffer_p = aq_imageinfo_p;
			//J_Image_FromRawToImageEx(pAqImageInfo, &m_tBuffer, BAYER_STANDARD_MULTI);
			//J_Image_FromRawToImagex(pAqImageInfo, &m_tBuffer, BAYER_STANDARD);
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
				std::cout << "Image size: " << aq_image_info_p->iImageSize << std::endl;
			};

			lock.unlock();
		return retval;

	}


	int JaiGenicamState::get_image(JaiGenicamConnection* wrapper, uint32_t* width_p, uint32_t* height_p, uint16_t* image_p)
	{
		int retval = -1;
		std::unique_lock<std::mutex> lock(wrapper->camera_mtx);		
			if (wrapper->image_buffer.pImageBuffer != NULL)
			{			
				*width_p = wrapper->image_buffer.iSizeX;
				*height_p = wrapper->image_buffer.iSizeY;
				
				std::copy(wrapper->image_buffer.pImageBuffer, 
					wrapper->image_buffer.pImageBuffer+(*width_p)*(*height_p)-1,
					image_p);
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


	/* -----------------------------------------------
		JaiGenicamConnectedState implementation
	--------------------------------------------------*/
	JaiGenicamConnectedState* JaiGenicamConnectedState::_instance = NULL;

	int JaiGenicamConnectedState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering CONNECTED state" << std::endl;
		wrapper->tango_ds_class->set_state(Tango::DevState::ON);

		int retval;
		retval = this->stop_capture(wrapper);
		retval = this->get_node(wrapper, &wrapper->gain_node);
		retval = this->get_node(wrapper, &wrapper->exposuretime_node);
		retval = this->get_node(wrapper, &wrapper->imagewidth_node);
		retval = this->get_node(wrapper, &wrapper->imageheight_node);
		retval = this->get_node(wrapper, &wrapper->pixelformat_node);
		retval = this->get_node(wrapper, &wrapper->framerate_node);
		double g;
		retval = this->get_gain(wrapper, &g);
		std::cout << "Gain value: " << g << std::endl;
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
		return 0;
	};


	template<typename T>
	int JaiGenicamConnectedState::get_node(JaiGenicamConnection* wrapper, GenicamNode<T>* node)
	{
//		std::cout << "JaiGenicamConnectedState::get_node for " << node->name << " of type " << node->type << std::endl;
		
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
//					std::cout << "Current value: " << int_value << std::endl;
					
					node->valid = true;
					break;
				}

			case J_IStringReg:			
			default:
				node->valid = false;
		}; // switch (node_type)
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
		if (node_type != J_IInteger)
		{
			err_msg <<  "Wrong type, should be " << J_IInteger << " (double), is " << node_type ;
			std::cout << err_msg.str() << std::endl;
			return -1;
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
	}


	/* -----------------------------------------------
		JaiGenicamRunningState implementation
	--------------------------------------------------*/
	JaiGenicamRunningState* JaiGenicamRunningState::_instance = NULL;

	int JaiGenicamRunningState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering RUNNING state" << std::endl;
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
		std::cout << "Starting aquisition" << std::endl;

		J_STATUS_TYPE   retval;
		std::stringstream err_msg;

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
		int64_t image_width = wrapper->imagewidth_node.value;
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
		
		return 0;
	};

	int JaiGenicamRunningState::stop_capture(JaiGenicamConnection* wrapper)
	{
		change_state(wrapper, JaiGenicamConnectedState::Instance());
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

	/* -----------------------------------------------
		JaiGenicamFaultState implementation
	--------------------------------------------------*/
	JaiGenicamFaultState* JaiGenicamFaultState::_instance = NULL;

	int JaiGenicamFaultState::enter( JaiGenicamConnection* wrapper )
	{
		std::cout << "Entering FAULT state" << std::endl;
		return 0;
	};

	int JaiGenicamFaultState::run(JaiGenicamConnection* wrapper)
	{
		wrapper->tango_state = wrapper->tango_ds_class->get_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return 0;
	};

} // namespace JaiGenicamConnection_ns