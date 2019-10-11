#include "JaiGenicamConnection.h"
#include <chrono>

/* ----------------------------------------
	JaiGenicamConnection implementation
-------------------------------------------*/


JaiGenicamConnection::JaiGenicamConnection(JaiGenicamDS_ns::JaiGenicamDS* tango_ds_class, std::string camera_serial)
{
	this->tango_ds_class = tango_ds_class;
	this->tango_state = this->tango_ds_class->get_state();

	this->camera_serial = camera_serial;
	this->camera_handle = NULL;
	this->factory_handle = NULL;

	this->jai_genicam_state = JaiGenicamDisconnectedState::Instance();
}


JaiGenicamConnection::~JaiGenicamConnection(void)
{
	close_camera();
	close_factory();
}

void JaiGenicamConnection::change_state(JaiGenicamState* new_state)
{
	this->jai_genicam_state = new_state;
	this->jai_genicam_state->enter(this);
};

void JaiGenicamConnection::start_thread()
{

};

void JaiGenicamConnection::stop_thread()
{

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
	this->jai_genicam_state->close_camera(this);
};

void JaiGenicamConnection::close_factory()
{
	this->jai_genicam_state->close_factory(this);
};


/* ---------------------------------
	JaiGenicamState implementation
------------------------------------*/

void JaiGenicamState::change_state( JaiGenicamConnection* c, JaiGenicamState* s)
{
	c->change_state(s);
};

void JaiGenicamState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering new state" << std::endl;
}


void JaiGenicamState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};

void JaiGenicamState::close_camera(JaiGenicamConnection* wrapper)
{
	J_STATUS_TYPE   retval;
	if (wrapper->camera_handle != NULL)
	{
		retval = J_Camera_Close(wrapper->camera_handle);
		if (retval != J_ST_SUCCESS)
		{
			std::cout <<  "Could not close camera! " << retval << std::endl;
		}
		else
		{
			std::cout << "Camera closed." << std::endl;
			wrapper->camera_handle = NULL;
		}
	}
	
};

void JaiGenicamState::close_factory(JaiGenicamConnection* wrapper)
{
	J_STATUS_TYPE   retval;
	if (wrapper->factory_handle != NULL)
	{
		retval = J_Factory_Close(wrapper->factory_handle);
		if (retval != J_ST_SUCCESS)
		{
			std::cout <<  "Could not close factory! " << retval << std::endl;
		}
		else
		{
			std::cout << "Factory closed." << std::endl;
			wrapper->factory_handle = NULL;
		}
	}
	
};

void JaiGenicamState::disconnect(JaiGenicamConnection* wrapper)
{
	close_camera(wrapper);
	close_factory(wrapper);
	change_state(wrapper, JaiGenicamDisconnectedState::Instance());
}

void JaiGenicamState::connect(JaiGenicamConnection* wrapper)
{

}



/* -----------------------------------------------
	JaiGenicamDisconnectedState implementation
--------------------------------------------------*/

void JaiGenicamDisconnectedState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering DISCONNECTED state" << std::endl;
}

void JaiGenicamDisconnectedState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};

void JaiGenicamDisconnectedState::connect(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();
	change_state(wrapper, JaiGenicamInitState::Instance());
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};

/* -----------------------------------------------
	JaiGenicamInitState implementation
--------------------------------------------------*/

void JaiGenicamInitState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering INIT state" << std::endl;
}

void JaiGenicamInitState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();

	J_STATUS_TYPE   retval;
	bool8_t         has_changed;
	uint32_t        n_cameras;
	int8_t          camera_id_s[J_CAMERA_ID_SIZE];
	uint32_t        size;

	retval = J_Factory_Open((int8_t*)"" , &wrapper->factory_handle);
	if (retval != J_ST_SUCCESS)
	{
		std::cout << "Could not open factory!" << retval << std::endl;
		return;
	}

	//Update camera list
	retval = J_Factory_UpdateCameraList(wrapper->factory_handle, &has_changed);
	if (retval != J_ST_SUCCESS)
	{
		std::cout << "Could not update camera list!" << retval << std::endl;
		return;
	}
	else
	{
		std::cout << "Camera list updated." << std::endl;
	}

	// Get the number of Cameras
	retval = J_Factory_GetNumOfCameras(wrapper->factory_handle, &n_cameras);
	if (retval != J_ST_SUCCESS)
	{
		std::cout << "Invalid number of cameras!" << retval << std::endl;
		return;
	}
	else
	{
		std::cout << "Found " << n_cameras << " cameras" << std::endl;
	}

	if (n_cameras == 0)
	{
		std::cout << "No cameras were found." << retval << std::endl;
		return;
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
			std::cout <<  "Could not get the camera ID!" << retval << std::endl;
			return;
		}
		else
		{
			std::cout << "Camera " << i << " ID: " << camera_id_s << std::endl;
		}

		retval = J_Factory_GetCameraInfo(wrapper->factory_handle, camera_id_s, CAM_INFO_SERIALNUMBER, s_camera_info, &size_ci);
		if (retval != J_ST_SUCCESS)
		{
			std::cout <<  "Could not get the camera info!" << retval << std::endl;
			return;
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
		return;
	}
	else
	{
		retval = J_Camera_Open(wrapper->factory_handle, camera_id_s, &wrapper->camera_handle);
		if (retval != J_ST_SUCCESS)
		{
			std::cout <<  "Could not open camera!" << retval << std::endl;
			return;
		}
		else
		{
			std::cout << "Camera open." << std::endl;
		}
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};


/* -----------------------------------------------
	JaiGenicamConnectedState implementation
--------------------------------------------------*/

void JaiGenicamConnectedState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering CONNECTED state" << std::endl;
}

void JaiGenicamConnectedState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};


/* -----------------------------------------------
	JaiGenicamRunningState implementation
--------------------------------------------------*/

void JaiGenicamRunningState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering RUNNING state" << std::endl;
}

void JaiGenicamRunningState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};


/* -----------------------------------------------
	JaiGenicamFaultState implementation
--------------------------------------------------*/

void JaiGenicamFaultState::enter( JaiGenicamConnection* wrapper )
{
	std::cout << "Entering FAULT state" << std::endl;
}

void JaiGenicamFaultState::run(JaiGenicamConnection* wrapper)
{
	wrapper->tango_state = wrapper->tango_ds_class->get_state();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
};
