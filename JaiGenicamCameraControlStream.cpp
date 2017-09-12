#include "JaiGenicamCameraControl.h"
#include <set>
#include <stack>
#include <VLib.h>

using namespace JaiGenicamCameraControl_ns;



/** Start data stream thread. If the was already opened, close it first.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::start_datastream()
{
	std::cout << "Starting data stream" << std::endl;

	J_STATUS_TYPE   retval;
	std::stringstream err_msg;

	// Close stream if already started
	retval = this->terminate_stream_thread();
	if (retval != 0)
	{
		err_msg << "Could not terminate stream thread! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
//		this->set_error_data("start_datastream", "terminate_stream_thread", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}

	std::unique_lock<std::mutex> lock(this->camera_mutex);	

	
	// Get the pixelformat from the camera
	uint64_t jaiPixelFormat = 0;
	int64_t remote_pixelformat;
	retval = this->get_node_value("PixelFormat", remote_pixelformat);

	retval = J_Image_Get_PixelFormat(this->camera_handle, remote_pixelformat, &jaiPixelFormat);
	if (retval != J_ST_SUCCESS) {
		err_msg << "Could not get pixelformat! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_datastream", "J_Image_Get_PixelFormat", err_msg.str(), retval, CameraState::NO_STATE, false);
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

	int8_t          transportlayer_s[J_FACTORY_INFO_SIZE];
	uint32_t        size;
	size = J_FACTORY_INFO_SIZE;
	retval = J_Camera_GetTransportLayerName(this->camera_handle, transportlayer_s, &size);
	std::cout << "J_Camera_GetTransportLayerName: retval " << this->get_error_string(retval) << ", string " << transportlayer_s << std::endl;

	//Make sure streaming is supported!
	uint32_t num_streams = 0;
	retval = J_Camera_GetNumOfDataStreams(this->camera_handle, &num_streams);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not get number of data streams! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
	}
	else
	{
		std::cout << num_streams << " data streams available." << std::endl;
	}

	int64_t payloadsize = 0;
	retval = J_Camera_GetValueInt64(this->camera_handle, NODE_NAME_PAYLOADSIZE, &payloadsize);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Unable to get PayloadSize value! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_datastream", "J_Camera_GetValueInt64", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}
	
	// Create data stream channel (GVSP)
	if (this->stream_handle == NULL)
	{		
		retval = J_Camera_CreateDataStream(this->camera_handle, 0, &this->stream_handle);
		if (retval != J_ST_SUCCESS) {
			err_msg << "Could not create data stream! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("start_datastream", "J_Camera_CreateDataStream", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		};
		if (this->stream_handle == NULL)
		{
			std::cout << "Stream handle NULL" << std::endl;
		}
		std::cout << "Data stream open " << std::endl;
	}

	// Prepare the frame buffers (this announces the buffers to the acquisition engine)
	uint32_t valid_buffers;
	std::cout << "Prepare buffers: " << std::endl;
	lock.unlock();
	retval = this->prepare_buffer(payloadsize, valid_buffers);
	if(retval != 0)
    {
		err_msg << "prepare_buffer failed! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
//		this->set_error_data("start_datastream", "prepare_buffer", err_msg.str(), retval, CameraState::NO_STATE, false);

		int retval2 = this->terminate_stream_thread();
		/*
		retval = J_DataStream_Close(this->stream_handle);
        if (retval != J_ST_SUCCESS)
        {
            err_msg << "Could not close data stream! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("start_datastream", "J_DataStream_Close", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
        }
		this->stream_handle = NULL;
		*/
        return retval;
    }

	// Stream thread event created?
	lock.lock();
	if(this->stream_event_handle == NULL)
	{
		std::cout << "start_datastream: CreateEvent" << std::endl;
        this->stream_event_handle = CreateEvent(NULL, true, false, NULL);
	}
    else
	{
		std::cout << "start_datastream: ResetEvent" << std::endl;
        ResetEvent(this->stream_event_handle);
	}

	// Start state handler thread
	std::cout << "start_datastream: thread creation" << std::endl;
	this->enable_thread = true;
	lock.unlock();
	this->stream_thread_handle = std::thread(&JaiGenicamCameraControl::stream_process, this);
	/*
	// Create a Stream Thread.
	if((this->stream_thread_handle = CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)ProcessCaller, this, NULL, NULL)) == NULL)
    {        
		err_msg << "CreateThread failed! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("start_datastream", "CreateThread", err_msg.str(), retval, CameraState::NO_STATE, false);

		retval = J_DataStream_Close(this->stream_handle);
        if (retval != J_ST_SUCCESS)
        {
            err_msg << "Could not close data stream! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("start_datastream", "J_DataStream_Close", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
        }
        return -1;
    }
	*/
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	lock.lock();
	// Start Acquisition
	J_NODE_ACCESSMODE access_mode;
	NODE_HANDLE node_handle;
	retval = J_Camera_GetNodeByName(this->camera_handle, NODE_NAME_ACQSTART, &node_handle);
	if (retval != J_ST_SUCCESS)
	{
		std::cout << "Acq stop get node handle fail " << this->get_error_string(retval) << std::endl;
	}
	retval = J_Node_GetAccessMode(node_handle, &access_mode);						
	if (retval != J_ST_SUCCESS)
	{
		std::cout << "Acq stop access mode fail " << this->get_error_string(retval) << std::endl;
	}
	else
	{
		switch (access_mode)
		{
		case J_NODE_ACCESSMODE::NA:
			std::cout << "Acq stop access NA (" << access_mode << ")" << std::endl;
			break;
		case J_NODE_ACCESSMODE::NI:
			std::cout << "Acq stop access NI (" << access_mode << ")" << std::endl;
			break;
		case J_NODE_ACCESSMODE::RO:
			std::cout << "Acq stop access RO (" << access_mode << ")" << std::endl;
			break;
		case J_NODE_ACCESSMODE::RW:
			std::cout << "Acq stop access RW (" << access_mode << ")" << std::endl;
			break;
		case J_NODE_ACCESSMODE::WO:
			std::cout << "Acq stop access WO (" << access_mode << ")" << std::endl;
			break;
		default:
			std::cout << "Acq stop access undefined (" << access_mode << ")" << std::endl;
			break;
		}
	}

	if ((access_mode == J_NODE_ACCESSMODE::RW) || (access_mode == J_NODE_ACCESSMODE::WO))
	{
		retval = J_Camera_ExecuteCommand(this->camera_handle, NODE_NAME_ACQSTART);
		if (retval != J_ST_SUCCESS)
		{
			switch (retval)
			{
			case J_ST_GC_ERROR:
				tGenICamErrorInfo gc;
				J_Factory_GetGenICamErrorInfo(&gc);
				err_msg << gc.sNodeName << " GC Error: Failure starting acquisition, returned " << gc.sDescription;
				break;
			default:
				err_msg << "Could not start aquisition! " << this->get_error_string(retval);
				std::cout << err_msg.str() << std::endl;
			}
			this->set_error_data("start_datastream", "J_Camera_ExecuteCommand", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
		std::cout << "Acquisition started" << std::endl;
	}
	else
	{
		std::cout << "Acq start access mode " << access_mode << std::endl;
	}
	lock.unlock();
	return 0;
}; // JaiGenicamCameraControl::start_datastream


/** Stop data stream thread.

Return 0 if successful, otherwise return the genicam error code.
*/
int JaiGenicamCameraControl::stop_datastream()
{
	std::cout << "Stopping data stream" << std::endl;

	J_STATUS_TYPE   retval;
	std::stringstream err_msg;

	std::unique_lock<std::mutex> lock(this->camera_mutex);

	if ((this->camera_handle != NULL) && (this->stream_handle != NULL))
	{
		retval = J_Camera_ExecuteCommand(this->camera_handle, NODE_NAME_ACQSTOP);
		if (retval != J_ST_SUCCESS)
		{
			switch (retval)
			{
			case J_ST_GC_ERROR:
				tGenICamErrorInfo gc;
				J_Factory_GetGenICamErrorInfo(&gc);
				err_msg << gc.sNodeName << " GC Error: Failure stopping acquisition, returned " << gc.sDescription;
				goto loop_exit;
				break;
			default:
				err_msg << "Could not Stop Acquisition! " << this->get_error_string(retval) << std::endl;
				std::cout << err_msg.str();			
			}
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("stop_camera_acquisition", "J_Camera_ExecuteCommand", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		};
	};
	loop_exit:
	lock.unlock();

	retval = this->terminate_stream_thread();
	if (retval != 0)
	{
		return retval;
	}

	return 0;
}; // JaiGenicamCameraControl::stop_datastream

/** ==============================================================
 Prepare frame buffers
============================================================== */
int JaiGenicamCameraControl::prepare_buffer(int buffersize, uint32_t &valid_buffers)
{
    J_STATUS_TYPE	retval = J_ST_SUCCESS;
	std::stringstream err_msg;
    int			i;

	std::cout << "Prepare buffer payload size " << buffersize << std::endl;
	std::lock_guard<std::mutex> lock(this->camera_mutex);

	this->valid_buffers = 0;

    for(i = 0 ; i < NUM_OF_BUFFERS ; i++)
    {
        // Make the buffer for one frame. 
		this->aq_buffer[i] = new uint8_t[buffersize];

        // Announce the buffer pointer to the Acquisition engine.
		std::cout << "Buffer " << i << std::endl;
		retval = J_DataStream_AnnounceBuffer(this->stream_handle, this->aq_buffer[i], (uint32_t)buffersize ,NULL, &(this->aq_buffer_id[i]));
		if(retval != J_ST_SUCCESS)
        {
			err_msg << "Could not Announce buffer! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("prepare_buffer", "J_DataStream_AnnounceBuffer", err_msg.str(), retval, CameraState::NO_STATE, false);			
		
            /*
			delete this->aq_buffer[i];
            break;
			*/
        }

        // Queueing it.
		if(J_ST_SUCCESS != J_DataStream_QueueBuffer(this->stream_handle, this->aq_buffer_id[i]))
        {
			std::stringstream err_msg;
			err_msg << "Could not Queue buffer! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("prepare_buffer", "J_DataStream_QueueBuffer", err_msg.str(), retval, CameraState::NO_STATE, false);			
		
            delete this->aq_buffer[i];
            break;
        }

        this->valid_buffers++;
    }
	valid_buffers = this->valid_buffers;
	if (valid_buffers > 0)
	{
		return 0;
	}
	else
	{
		return -1;
	};
}; // JaiGenicamCameraControl::prepare_buffer


//==============================================================////
// Unprepare buffers
//==============================================================////
int JaiGenicamCameraControl::unprepare_buffer(void)
{
    void		*private_data;
    void		*buffer;
    uint32_t	i;

	std::lock_guard<std::mutex> lock(this->camera_mutex);	

    // Flush Queues
	std::cout << "unprepare_buffer: J_DataStream_FlushQueue" << std::endl;
	J_DataStream_FlushQueue(this->stream_handle, ACQ_QUEUE_INPUT_TO_OUTPUT);
    J_DataStream_FlushQueue(this->stream_handle, ACQ_QUEUE_OUTPUT_DISCARD);

	for(i = 0 ; i < this->valid_buffers ; i++)
    {
        // Remove the frame buffer from the Acquisition engine.
		J_DataStream_RevokeBuffer(this->stream_handle, this->aq_buffer_id[i], &buffer , &private_data);

		delete this->aq_buffer[i];
        this->aq_buffer[i] = NULL;
        this->aq_buffer_id[i] = 0;
    }

    this->valid_buffers = 0;

    return 0;
} // JaiGenicamCameraControl::unprepare_buffer


//==============================================================////
// Terminate Stream Thread
//==============================================================////
int JaiGenicamCameraControl::terminate_stream_thread(void)
{
    J_STATUS_TYPE   retval;
	std::stringstream err_msg;

	std::cout << "terminate_stream_thread:" << std::endl;

	std::unique_lock<std::mutex> lock(this->camera_mutex);

    // Is the data stream opened?
	if(this->stream_handle == NULL)
        return 0;

    // Reset the thread execution flag.
	this->enable_thread = false;

    // Signal the image thread to stop faster
	if (this->event_condition_handle != NULL)
    {
		std::cout << "terminate_stream_thread: J_Event_ExitCondition" << std::endl;
        retval = J_Event_ExitCondition(this->event_condition_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not exit Condition! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("terminate_stream_thread", "J_Event_ExitCondition", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		};
    };
	lock.unlock();

	lock.lock();
    // Stop the image acquisition engine
	std::cout << "terminate_stream_thread: J_DataStream_StopAcquisition" << std::endl;
    retval = J_DataStream_StopAcquisition(this->stream_handle, ACQ_STOP_FLAG_KILL);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not stop DataStream acquisition! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("terminate_stream_thread", "J_DataStream_StopAcquisition", err_msg.str(), retval, CameraState::NO_STATE, false);
		return retval;
	}

    // Mark stream acquisition as stopped
	this->stream_started = false;

	lock.unlock();
    // Wait for the thread to end
    this->wait_for_thread_to_terminate();

    // UnPrepare Buffers (this removed the buffers from the acquisition engine and frees buffers)
    this->unprepare_buffer();

	lock.lock();
    // Close Stream
    if(this->stream_handle != NULL)
    {
        retval = J_DataStream_Close(this->stream_handle);
		if (retval != J_ST_SUCCESS)
		{
			err_msg << "Could not close DataStream! " << this->get_error_string(retval);
			std::cout << err_msg.str() << std::endl;
			this->set_error_data("terminate_stream_thread", "J_DataStream_Close", err_msg.str(), retval, CameraState::NO_STATE, false);
			return retval;
		}
        this->stream_handle = NULL;
    }

	this->stream_thread_handle.join();
    return 0;
} // JaiGenicamCameraControl::terminate_stream_thread


/**==============================================================
 Stream Processing Function
==============================================================*/
void JaiGenicamCameraControl::stream_process(void)
{
	std::cout << "stream_process: " << std::endl;
    J_STATUS_TYPE	retval;
	std::stringstream err_msg;
    uint32_t        size;
    BUF_HANDLE	    buffer_id;
	uint64_t	    queued = 0;

    // Create structure to be used for image display
    J_tIMAGE_INFO	aq_image_info = {0, 0, 0, 0, NULL, 0, 0, 0, 0, 0, 0};

    J_COND_WAIT_RESULT	wait_result;

	std::unique_lock<std::mutex> lock(this->camera_mutex);
    // Create the condition used for signaling the new image event
	retval = J_Event_CreateCondition(&this->event_condition_handle);

    EVT_HANDLE	event_handle;					// Buffer event handle
	EVENT_NEW_BUFFER_DATA event_data;		// Struct for EventGetData

	
	// Register the event with the acquisition engine
	uint32_t acquisition_flag = ACQ_START_NEXT_IMAGE;

	retval = J_DataStream_RegisterEvent(this->stream_handle, EVENT_NEW_BUFFER, this->event_condition_handle, &event_handle); 
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not register event! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("stream_process", "J_DataStream_RegisterEvent", err_msg.str(), retval, CameraState::NO_STATE, false);
	}

    // Start image acquisition
    retval = J_DataStream_StartAcquisition(this->stream_handle, acquisition_flag, ULLONG_MAX);
	if (retval != J_ST_SUCCESS)
	{
		err_msg << "Could not start stream acquisition! " << this->get_error_string(retval);
		std::cout << err_msg.str() << std::endl;
		this->set_error_data("stream_process", "J_DataStream_StartAcquisition", err_msg.str(), retval, CameraState::NO_STATE, false);
	}

    // Mark stream acquisition as started
	this->stream_started = true;

    // Loop of Stream Processing
    std::cout << ">>> Start Stream Process Loop." << std::endl;

	const uint64_t timeout = 1000;
	uint64_t jai_pixelformat = 0;
	uint64_t chunk_layout_id_value = 0; // 0 means no Chunk
	uint64_t last_chunk_layout_id = 0;
	size_t num_chunks = 0;
	size_t last_num_chunks = 0;
	J_SINGLE_CHUNK_DATA* chunk_infoarray = NULL;

	bool enable_thread = this->enable_thread;
	lock.unlock();

	

	while(enable_thread)
    {
				
        // Wait for Buffer event (or kill event)
		retval = J_Event_WaitForCondition(this->event_condition_handle, timeout, &wait_result);
		if(J_ST_SUCCESS != retval)
		{
			std::cout << "J_Event_WaitForCondition Error." << std::endl;
		}
		// Did we get a new buffer event?
        if(wait_result == J_COND_WAIT_SIGNAL)
        {
            // Get the Buffer Handle from the event
            size = (uint32_t)sizeof(EVENT_NEW_BUFFER_DATA);

			retval = J_Event_GetData(event_handle, &event_data,  &size);
			if(retval != J_ST_SUCCESS)
			{
				std::cout << "J_Event_GetData Error." << std::endl;
			}

			buffer_id = event_data.BufferHandle;

            // Did we receive the event data?
            if (retval == J_ST_SUCCESS)
            {
                // Fill in structure for image display

                // Get the pointer to the frame buffer.
				uint64_t info_value = 0;
                size = (uint32_t)sizeof (void *);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_BASE, &(info_value), &size);
				aq_image_info.pImageBuffer = (uint8_t*)info_value;
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_BASE." << std::endl;
					continue;
				}
                
				// Get the effective data size.
				size = sizeof (size_t);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_SIZE, &(info_value), &size);
				aq_image_info.iImageSize = (uint32_t)info_value;
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_SIZE." << std::endl;
					continue;
				}

				
				// Get Frame Width.
				size = sizeof (size_t);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_WIDTH	, &(info_value), &size);
				aq_image_info.iSizeX = (uint32_t)info_value;
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_WIDTH." << std::endl;
					continue;
				}
                
				// Get Frame Height.
				size = sizeof (size_t);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_HEIGHT	, &(info_value), &size);
				aq_image_info.iSizeY = (uint32_t)info_value;
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_HEIGHT." << std::endl;
					continue;
				}
                
				// Get Pixel Format Type.
				try
				{
					size = sizeof (uint64_t);
					retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_PIXELTYPE, &(info_value), &size);
					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_PIXELTYPE." << std::endl;
						aq_image_info.iPixelType = GVSP_PIX_MONO8;
					}
					
					//Convert the camera's pixel format value to one understood by the JAI SDK.
					lock.lock();
					retval = J_Image_Get_PixelFormat(this->camera_handle, info_value, &jai_pixelformat);
					lock.unlock();

					aq_image_info.iPixelType = jai_pixelformat;

					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_Image_Get_PixelFormat in stream_process ==> BUFFER_INFO_PIXELTYPE." << std::endl;
						aq_image_info.iPixelType = GVSP_PIX_MONO8;
					}
				}
				catch (...)
				{
					std::cout << "Exception with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_PIXELTYPE - assuming GVSP_PIX_MONO8." << std::endl;
					aq_image_info.iPixelType = GVSP_PIX_MONO8;
				}

				// Get Timestamp.
				try
				{
					size = sizeof (info_value);
					retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_TIMESTAMP, &(info_value), &size);
					aq_image_info.iTimeStamp = info_value;
					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_TIMESTAMP." << std::endl;
						//continue;
					}
				}
				catch(...)
				{
					std::cout << "Exception with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_TIMESTAMP." << std::endl;
				}

				// Get # of missing packets in frame.
				size = (uint32_t)sizeof (uint32_t);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_NUM_PACKETS_MISSING, &(aq_image_info.iMissingPackets), &size);                

                // Initialize number of valid buffers announced
				aq_image_info.iAnnouncedBuffers = this->valid_buffers;

                // Get # of buffers queued
				size = sizeof (info_value);
				retval = J_DataStream_GetStreamInfo(this->stream_handle, STREAM_INFO_CMD_NUMBER_OF_FRAMES_QUEUED, &queued, &size);
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> STREAM_INFO_NUM_QUEUED." << std::endl;
					//continue;
				}

				aq_image_info.iQueuedBuffers = static_cast<uint32_t>(queued & 0x0ffffffffL);

				// Get X-offset.
				try
				{
					size = sizeof (size_t);
					retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_XOFFSET, &info_value, &size);
					aq_image_info.iOffsetX = (uint32_t)(info_value);
					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_XOFFSET." << std::endl;
						//continue;
					}
				}
				catch(...)
				{
					std::cout << "Exception with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_XOFFSET." << std::endl;
				}
                
				// Get Y-offset.
				try
				{
					size = sizeof (size_t);
					uint64_t infoValue = 0;
					retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_YOFFSET, &info_value, &size);
					aq_image_info.iOffsetY = (uint32_t)(info_value);
					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_YOFFSET." << std::endl;
						//continue;
					}
				}
				catch(...)
				{
					std::cout << "Exception with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_YOFFSET." << std::endl;
					//continue;
				}
                
				// Get Block ID
				size = sizeof (uint64_t);
				retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_BLOCKID, &(aq_image_info.iBlockId), &size);
				if(GC_ERR_SUCCESS != retval)
				{
					std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_BLOCKID." << std::endl;
					//continue;
				}

				
				// Get Chunk Layout ID.
				chunk_layout_id_value = 0;
				try
				{
					size = sizeof (chunk_layout_id_value);
					retval = J_DataStream_GetBufferInfo(this->stream_handle, buffer_id, BUFFER_INFO_CHUNKLAYOUTID, &(chunk_layout_id_value), &size);
					if(GC_ERR_SUCCESS != retval)
					{
						std::cout << "Error with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_CHUNKLAYOUTID." << std::endl;
						//continue;
					}
					else
					{
						// OK - we got Chunk Layout ID number so we can check whether it has changed!
						// If it changes then we will have to get the new Chunk Information from the datastream
						if (chunk_layout_id_value != last_chunk_layout_id)
						{
							last_chunk_layout_id = chunk_layout_id_value;
							
							// First we have to get the current number of chunks
							num_chunks = 0;
							retval = J_DataStream_GetBufferChunkData(this->stream_handle, buffer_id, NULL, &num_chunks);
							if(GC_ERR_SUCCESS != retval)
							{
								std::cout << "Error with J_DataStream_GetBufferChunkData in stream_process." << std::endl;
							}
							else
							{
								// Did the number of chunks change? 
								// Then we would have to re-allocate the Chunk Info structure
								if (num_chunks != last_num_chunks)
								{
									std::cout << "stream_process: Number of chunks changed." << std::endl;
									last_num_chunks = num_chunks;

									if (chunk_infoarray != NULL)
										delete chunk_infoarray;
									chunk_infoarray = (J_SINGLE_CHUNK_DATA*) malloc (num_chunks*sizeof(J_SINGLE_CHUNK_DATA));
								}

								retval = J_DataStream_GetBufferChunkData(this->stream_handle, buffer_id, chunk_infoarray, &num_chunks);
								if(GC_ERR_SUCCESS != retval)
								{
									std::cout << "Error with J_DataStream_GetBufferChunkData in stream_process ==> ." << std::endl;
								}
								else
								{
									// Now we have all the chunk information we need!
									lock.lock();
									std::cout << "stream_process: J_Camera_AttachChunkData." << std::endl;
									retval = J_Camera_AttachChunkData(this->camera_handle, aq_image_info.pImageBuffer, chunk_infoarray, num_chunks);
									if(GC_ERR_SUCCESS != retval)
									{
										std::cout << "Error with J_Camera_AttachChunkData in stream_process." << std::endl;
									}
									lock.unlock();
								}
							}
						}
						else
						{
							// OK - here we can assume that the chunk layout is unchanges so we only need to update the chunk data
							lock.lock();
							retval = J_Camera_UpdateChunkData(this->camera_handle, aq_image_info.pImageBuffer);
							if(GC_ERR_SUCCESS != retval)
							{
								switch (retval)
								{
								case J_ST_INVALID_PARAMETER:
									break;
								default:
									std::cout << "stream_process: Error with J_Camera_UpdateChunkData, returned " << this->get_error_string(retval) << std::endl;
								}
							}
							lock.unlock();
						}
					}
				}
				catch(...)
				{
					std::cout << "Exception with J_DataStream_GetBufferInfo in stream_process ==> BUFFER_INFO_CHUNKLAYOUTID." << std::endl;
				}

				// Just testing Chunk
				int64_t chunk_data_value = 0;
				if (chunk_layout_id_value != 0) 
				{
					J_STATUS_TYPE err = J_ST_SUCCESS;

					// If this is "pure" chunk data then we probably need to get the Image information from the chunk data!
					if ((aq_image_info.iSizeX == 0) && (aq_image_info.iSizeY == 0))
					{
						std::cout << "stream_process: 'pure' chunk data"  << std::endl;
						lock.lock();

						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkWidth", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iSizeX = (uint32_t)chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}

						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkHeight", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iSizeY = (uint32_t)chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}

						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkHeight", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iSizeY = (uint32_t)chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}

						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkOffsetX", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iOffsetX = (uint32_t)chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}
						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkOffsetY", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iOffsetY = (uint32_t)chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}
						err = J_Camera_GetValueInt64(this->camera_handle, (int8_t*)"ChunkPixelFormat", &chunk_data_value);

						if (err == J_ST_SUCCESS)
						{
							aq_image_info.iPixelType = chunk_data_value;
							//WCHAR buffer[100];
							//wsprintf(buffer, _T("Chunk Timestamp=%I64u\n"), chunkDataValue);
							//OutputDebugString(buffer);
						}
						lock.unlock();
					}
				}
				if(enable_thread)
                {
                    // Copy image
					double frame_time;
					int new_framecounter;
					frame_time = (std::chrono::duration_cast<std::chrono::milliseconds>
						(std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
					// The state of the buffer is confirmed.

					lock.lock();

					if(this->image_buffer.pImageBuffer != NULL)
					{
						// When the buffer already exists, and the size is different:
						if((this->image_buffer.iSizeX != aq_image_info.iSizeX) || (this->image_buffer.iSizeY != aq_image_info.iSizeY) || (this->image_buffer.iPixelType != aq_image_info.iPixelType))
						{
							// Abandons the buffer.
							retval = J_Image_Free(&this->image_buffer);
							if (retval != J_ST_SUCCESS)
							{
								err_msg << "Could not free buffer!" << this->get_error_string(retval);
								std::cout << err_msg.str() << std::endl;
								this->set_error_data("capture_stream_callback", "J_Image_Free", err_msg.str(), retval, CameraState::NO_STATE, false);
							};
							this->image_buffer.pImageBuffer = NULL;
						}
					}

					// Allocates it when there is no buffer.
					if(this->image_buffer.pImageBuffer == NULL)
					{
						retval = J_Image_Malloc(&aq_image_info, &this->image_buffer);
						if (retval != J_ST_SUCCESS)
						{
							err_msg << "Could not allocate buffer!" << this->get_error_string(retval);
							std::cout << err_msg.str() << std::endl;
							this->set_error_data("capture_stream_callback", "J_Image_Malloc", err_msg.str(), retval, CameraState::NO_STATE, false);
						};
					};

					// The image making is done for the picture processing.
					retval = J_Image_FromRawToImage(&aq_image_info, &this->image_buffer);
					if (retval != J_ST_SUCCESS)
					{
						err_msg << "Could not convert to RAW!" << this->get_error_string(retval);
						std::cout << err_msg.str() << std::endl;
						this->set_error_data("capture_stream_callback", "J_Image_FromRawToImage", err_msg.str(), retval, CameraState::NO_STATE, false);
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
                    
                }

                // Queue This Buffer Again for reuse in acquisition engine
				retval = J_DataStream_QueueBuffer(this->stream_handle, buffer_id);
            
			} // if (iResult == J_ST_SUCCESS) J_Event_GetData

            // Get # of frames awaiting delivery
            size = sizeof(size_t);
            retval = J_DataStream_GetStreamInfo(this->stream_handle, STREAM_INFO_CMD_NUMBER_OF_FRAMES_AWAIT_DELIVERY, &queued, &size);
			
		} // if(wait_result == J_COND_WAIT_SIGNAL)
		else
        {
            switch(wait_result)
            {
                // Kill event
                case	J_COND_WAIT_EXIT:
					std::cout << "stream_process: >>> Wait error J_COND_WAIT_EXIT " << std::endl;
					retval = 1;
					break;
                // Timeout
                case	J_COND_WAIT_TIMEOUT:
					std::cout << "stream_process: >>> Wait error J_COND_WAIT_TIMEOUT " << std::endl;
					retval = 2;
					break;
                // Error event
                case	J_COND_WAIT_ERROR:
					std::cout << "stream_process: >>> Wait error J_COND_WAIT_ERROR " << std::endl;
					retval = 3;
					break;
                // Unknown?
                default:
					std::cout << "stream_process: >>> Wait error Unknown " << std::endl;
					retval = 4;
					break;
            }
        }
		lock.lock();
		enable_thread = this->enable_thread;
		lock.unlock();
	} // while

	std::cout << "stream_process: J_Camera_DetachChunkData"  << std::endl;
	// OK - chunk data might have been used so we want to detach any buffers and Chunk Info structures
	retval = J_Camera_DetachChunkData(this->camera_handle);
	if(GenICam::Client::GC_ERR_SUCCESS != retval)
	{
		std::cout << "stream_process: Error with J_Camera_DetachChunkData " << std::endl;
	}

    // Unregister new buffer event with acquisition engine
	std::cout << "stream_process: J_DataStream_UnRegisterEvent"  << std::endl;
	retval = J_DataStream_UnRegisterEvent(this->stream_handle, EVENT_NEW_BUFFER); 

    // Free the event object
    if (event_handle != NULL)
    {
        retval = J_Event_Close(event_handle);
        event_handle = NULL;
    }

    // Terminate the thread.
	std::cout << "stream_process: SetEvent"  << std::endl;
	SetEvent(this->stream_event_handle);

    // Free the Condition
	std::cout << "stream_process: J_Event_CloseCondition"  << std::endl;
	if (this->event_condition_handle != NULL)
    {
        retval = J_Event_CloseCondition(this->event_condition_handle);
        this->event_condition_handle = NULL;
    }

	// Free the Chunk Info data
	
	if (chunk_infoarray != NULL)
	{
		delete chunk_infoarray;
		chunk_infoarray = NULL;
	}
	
	std::cout << "stream_process: exit"  << std::endl;
}; // JaiGenicamCameraControl::stream_process


/** ==============================================================
 Wait for thread to terminate
==============================================================*/
void JaiGenicamCameraControl::wait_for_thread_to_terminate(void)
{
	std::cout << "wait_for_thread_to_terminate" << std::endl;
	WaitForSingleObject(this->stream_event_handle, 3000);
	std::cout << "wait_for_thread_to_terminate done" << std::endl;
    // Close the thread handle and stream event handle
	this->close_thread_handle();
} // JaiGenicamCameraControl::wait_for_thread_to_terminate


/** ==============================================================
 Close handles and stream
==============================================================*/
void JaiGenicamCameraControl::close_thread_handle(void)
{	
	if(this->stream_event_handle)
    {
        CloseHandle(this->stream_event_handle);
        this->stream_event_handle = NULL;
    }
} // JaiGenicamCameraControl::close_thread_handle