/*----- PROTECTED REGION ID(JaiGenicamDS.h) ENABLED START -----*/
//=============================================================================
//
// file :        JaiGenicamDS.h
//
// description : Include file for the JaiGenicamDS class
//
// project :     JaiGenicamDS
//
// This file is part of Tango device class.
// 
// Tango is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Tango is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Tango.  If not, see <http://www.gnu.org/licenses/>.
// 
// $Author:  $
//
// $Revision:  $
// $Date:  $
//
// $HeadURL:  $
//
//=============================================================================
//                This file is generated by POGO
//        (Program Obviously used to Generate tango Object)
//=============================================================================


#ifndef JaiGenicamDS_H
#define JaiGenicamDS_H

#include <tango.h>
namespace JaiGenicamConnection_ns
	{
		class JaiGenicamConnection;	
	};

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDS.h

/**
 *  JaiGenicamDS class description:
 *    Control of Jai Genicam cameras
 */

namespace JaiGenicamDS_ns
{
/*----- PROTECTED REGION ID(JaiGenicamDS::Additional Class Declarations) ENABLED START -----*/

//	Additional Class Declarations
	

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDS::Additional Class Declarations

class JaiGenicamDS : public TANGO_BASE_CLASS
{

/*----- PROTECTED REGION ID(JaiGenicamDS::Data Members) ENABLED START -----*/

//	Add your own data members

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDS::Data Members

//	Device property data members
public:
	//	serial_number:	Serial number of the camera, used to identify it on the network
	string	serial_number;

	bool	mandatoryNotDefined;

//	Attribute data members
public:
	Tango::DevDouble	*attr_ExposureTime_read;
	Tango::DevDouble	*attr_Gain_read;
	Tango::DevDouble	*attr_FrameRate_read;
	Tango::DevBoolean	*attr_ExternalTrigger_read;
	Tango::DevLong64	*attr_FrameCounter_read;
	Tango::DevUShort	*attr_Image_read;

//	Constructors and destructors
public:
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	JaiGenicamDS(Tango::DeviceClass *cl,string &s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	JaiGenicamDS(Tango::DeviceClass *cl,const char *s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device name
	 *	@param d	Device description.
	 */
	JaiGenicamDS(Tango::DeviceClass *cl,const char *s,const char *d);
	/**
	 * The device object destructor.
	 */	
	~JaiGenicamDS() {delete_device();};


//	Miscellaneous methods
public:
	/*
	 *	will be called at device destruction or at init command.
	 */
	void delete_device();
	/*
	 *	Initialize the device
	 */
	virtual void init_device();
	/*
	 *	Read the device properties from database
	 */
	void get_device_property();
	/*
	 *	Always executed method before execution command method.
	 */
	virtual void always_executed_hook();

	/*
	 *	Check if mandatory property has been set
	 */
	 void check_mandatory_property(Tango::DbDatum &class_prop, Tango::DbDatum &dev_prop);

//	Attribute methods
public:
	//--------------------------------------------------------
	/*
	 *	Method      : JaiGenicamDS::read_attr_hardware()
	 *	Description : Hardware acquisition for attributes.
	 */
	//--------------------------------------------------------
	virtual void read_attr_hardware(vector<long> &attr_list);
	//--------------------------------------------------------
	/*
	 *	Method      : JaiGenicamDS::write_attr_hardware()
	 *	Description : Hardware writing for attributes.
	 */
	//--------------------------------------------------------
	virtual void write_attr_hardware(vector<long> &attr_list);

/**
 *	Attribute ExposureTime related methods
 *	Description: Exposure time of the capture in milliseconds
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_ExposureTime(Tango::Attribute &attr);
	virtual void write_ExposureTime(Tango::WAttribute &attr);
	virtual bool is_ExposureTime_allowed(Tango::AttReqType type);
/**
 *	Attribute Gain related methods
 *	Description: Camera gain value. The unit depends on the specific camera implementation.
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_Gain(Tango::Attribute &attr);
	virtual void write_Gain(Tango::WAttribute &attr);
	virtual bool is_Gain_allowed(Tango::AttReqType type);
/**
 *	Attribute FrameRate related methods
 *	Description: Frame rate of the camera when free running
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_FrameRate(Tango::Attribute &attr);
	virtual void write_FrameRate(Tango::WAttribute &attr);
	virtual bool is_FrameRate_allowed(Tango::AttReqType type);
/**
 *	Attribute ExternalTrigger related methods
 *	Description: Select if external (true) or internal (false) triggering is to be used.
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_ExternalTrigger(Tango::Attribute &attr);
	virtual void write_ExternalTrigger(Tango::WAttribute &attr);
	virtual bool is_ExternalTrigger_allowed(Tango::AttReqType type);
/**
 *	Attribute FrameCounter related methods
 *	Description: Counts number of frames captured since starting acquisition. 
 *
 *	Data type:	Tango::DevLong64
 *	Attr type:	Scalar
 */
	virtual void read_FrameCounter(Tango::Attribute &attr);
	virtual bool is_FrameCounter_allowed(Tango::AttReqType type);
/**
 *	Attribute Image related methods
 *	Description: Latest captured image
 *
 *	Data type:	Tango::DevUShort
 *	Attr type:	Image max = 4096 x 4096
 */
	virtual void read_Image(Tango::Attribute &attr);
	virtual bool is_Image_allowed(Tango::AttReqType type);


	//--------------------------------------------------------
	/**
	 *	Method      : JaiGenicamDS::add_dynamic_attributes()
	 *	Description : Add dynamic attributes if any.
	 */
	//--------------------------------------------------------
	void add_dynamic_attributes();



//	Command related methods
public:
	/**
	 *	Command start related method
	 *	Description: Start camera acquisition
	 *
	 */
	virtual void start();
	virtual bool is_start_allowed(const CORBA::Any &any);
	/**
	 *	Command stop related method
	 *	Description: Stop camera acquisition
	 *
	 */
	virtual void stop();
	virtual bool is_stop_allowed(const CORBA::Any &any);
	/**
	 *	Command Off related method
	 *	Description: Disconnect from camera (e.g. to allow other software to use it)
	 *
	 */
	virtual void off();
	virtual bool is_Off_allowed(const CORBA::Any &any);
	/**
	 *	Command On related method
	 *	Description: Connect to camera
	 *
	 */
	virtual void on();
	virtual bool is_On_allowed(const CORBA::Any &any);


/*----- PROTECTED REGION ID(JaiGenicamDS::Additional Method prototypes) ENABLED START -----*/

//	Additional Method prototypes
private:
	::JaiGenicamConnection_ns::JaiGenicamConnection* camera_connection;

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDS::Additional Method prototypes
};

/*----- PROTECTED REGION ID(JaiGenicamDS::Additional Classes Definitions) ENABLED START -----*/

//	Additional Classes Definitions

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDS::Additional Classes Definitions

}	//	End of namespace

#endif   //	JaiGenicamDS_H
