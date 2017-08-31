/*----- PROTECTED REGION ID(JaiGenicamDSClass.cpp) ENABLED START -----*/
static const char *RcsId      = "$Id:  $";
static const char *TagName    = "$Name:  $";
static const char *CvsPath    = "$Source:  $";
static const char *SvnPath    = "$HeadURL:  $";
static const char *HttpServer = "http://www.esrf.eu/computing/cs/tango/tango_doc/ds_doc/";
//=============================================================================
//
// file :        JaiGenicamDSClass.cpp
//
// description : C++ source for the JaiGenicamDSClass.
//               A singleton class derived from DeviceClass.
//               It implements the command and attribute list
//               and all properties and methods required
//               by the JaiGenicamDS once per process.
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


#include <JaiGenicamDSClass.h>

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass.cpp

//-------------------------------------------------------------------
/**
 *	Create JaiGenicamDSClass singleton and
 *	return it in a C function for Python usage
 */
//-------------------------------------------------------------------
extern "C" {
#ifdef _TG_WINDOWS_

__declspec(dllexport)

#endif

	Tango::DeviceClass *_create_JaiGenicamDS_class(const char *name) {
		return JaiGenicamDS_ns::JaiGenicamDSClass::init(name);
	}
}

namespace JaiGenicamDS_ns
{
//===================================================================
//	Initialize pointer for singleton pattern
//===================================================================
JaiGenicamDSClass *JaiGenicamDSClass::_instance = NULL;

//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::JaiGenicamDSClass(string &s)
 * description : 	constructor for the JaiGenicamDSClass
 *
 * @param s	The class name
 */
//--------------------------------------------------------
JaiGenicamDSClass::JaiGenicamDSClass(string &s):Tango::DeviceClass(s)
{
	cout2 << "Entering JaiGenicamDSClass constructor" << endl;
	set_default_property();
	write_class_property();

	/*----- PROTECTED REGION ID(JaiGenicamDSClass::constructor) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::constructor

	cout2 << "Leaving JaiGenicamDSClass constructor" << endl;
}

//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::~JaiGenicamDSClass()
 * description : 	destructor for the JaiGenicamDSClass
 */
//--------------------------------------------------------
JaiGenicamDSClass::~JaiGenicamDSClass()
{
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::destructor) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::destructor

	_instance = NULL;
}


//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::init
 * description : 	Create the object if not already done.
 *                  Otherwise, just return a pointer to the object
 *
 * @param	name	The class name
 */
//--------------------------------------------------------
JaiGenicamDSClass *JaiGenicamDSClass::init(const char *name)
{
	if (_instance == NULL)
	{
		try
		{
			string s(name);
			_instance = new JaiGenicamDSClass(s);
		}
		catch (bad_alloc &)
		{
			throw;
		}		
	}		
	return _instance;
}

//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::instance
 * description : 	Check if object already created,
 *                  and return a pointer to the object
 */
//--------------------------------------------------------
JaiGenicamDSClass *JaiGenicamDSClass::instance()
{
	if (_instance == NULL)
	{
		cerr << "Class is not initialised !!" << endl;
		exit(-1);
	}
	return _instance;
}



//===================================================================
//	Command execution method calls
//===================================================================
//--------------------------------------------------------
/**
 * method : 		startClass::execute()
 * description : 	method to trigger the execution of the command.
 *
 * @param	device	The device on which the command must be executed
 * @param	in_any	The command input data
 *
 *	returns The command output data (packed in the Any object)
 */
//--------------------------------------------------------
CORBA::Any *startClass::execute(Tango::DeviceImpl *device, TANGO_UNUSED(const CORBA::Any &in_any))
{
	cout2 << "startClass::execute(): arrived" << endl;
	((static_cast<JaiGenicamDS *>(device))->start());
	return new CORBA::Any();
}

//--------------------------------------------------------
/**
 * method : 		stopClass::execute()
 * description : 	method to trigger the execution of the command.
 *
 * @param	device	The device on which the command must be executed
 * @param	in_any	The command input data
 *
 *	returns The command output data (packed in the Any object)
 */
//--------------------------------------------------------
CORBA::Any *stopClass::execute(Tango::DeviceImpl *device, TANGO_UNUSED(const CORBA::Any &in_any))
{
	cout2 << "stopClass::execute(): arrived" << endl;
	((static_cast<JaiGenicamDS *>(device))->stop());
	return new CORBA::Any();
}

//--------------------------------------------------------
/**
 * method : 		OffClass::execute()
 * description : 	method to trigger the execution of the command.
 *
 * @param	device	The device on which the command must be executed
 * @param	in_any	The command input data
 *
 *	returns The command output data (packed in the Any object)
 */
//--------------------------------------------------------
CORBA::Any *OffClass::execute(Tango::DeviceImpl *device, TANGO_UNUSED(const CORBA::Any &in_any))
{
	cout2 << "OffClass::execute(): arrived" << endl;
	((static_cast<JaiGenicamDS *>(device))->off());
	return new CORBA::Any();
}

//--------------------------------------------------------
/**
 * method : 		OnClass::execute()
 * description : 	method to trigger the execution of the command.
 *
 * @param	device	The device on which the command must be executed
 * @param	in_any	The command input data
 *
 *	returns The command output data (packed in the Any object)
 */
//--------------------------------------------------------
CORBA::Any *OnClass::execute(Tango::DeviceImpl *device, TANGO_UNUSED(const CORBA::Any &in_any))
{
	cout2 << "OnClass::execute(): arrived" << endl;
	((static_cast<JaiGenicamDS *>(device))->on());
	return new CORBA::Any();
}

//--------------------------------------------------------
/**
 * method : 		GetCameraListClass::execute()
 * description : 	method to trigger the execution of the command.
 *
 * @param	device	The device on which the command must be executed
 * @param	in_any	The command input data
 *
 *	returns The command output data (packed in the Any object)
 */
//--------------------------------------------------------
CORBA::Any *GetCameraListClass::execute(Tango::DeviceImpl *device, TANGO_UNUSED(const CORBA::Any &in_any))
{
	cout2 << "GetCameraListClass::execute(): arrived" << endl;
	return insert((static_cast<JaiGenicamDS *>(device))->get_camera_list());
}


//===================================================================
//	Properties management
//===================================================================
//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::get_class_property()
 *	Description : Get the class property for specified name.
 */
//--------------------------------------------------------
Tango::DbDatum JaiGenicamDSClass::get_class_property(string &prop_name)
{
	for (unsigned int i=0 ; i<cl_prop.size() ; i++)
		if (cl_prop[i].name == prop_name)
			return cl_prop[i];
	//	if not found, returns  an empty DbDatum
	return Tango::DbDatum(prop_name);
}

//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::get_default_device_property()
 *	Description : Return the default value for device property.
 */
//--------------------------------------------------------
Tango::DbDatum JaiGenicamDSClass::get_default_device_property(string &prop_name)
{
	for (unsigned int i=0 ; i<dev_def_prop.size() ; i++)
		if (dev_def_prop[i].name == prop_name)
			return dev_def_prop[i];
	//	if not found, return  an empty DbDatum
	return Tango::DbDatum(prop_name);
}

//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::get_default_class_property()
 *	Description : Return the default value for class property.
 */
//--------------------------------------------------------
Tango::DbDatum JaiGenicamDSClass::get_default_class_property(string &prop_name)
{
	for (unsigned int i=0 ; i<cl_def_prop.size() ; i++)
		if (cl_def_prop[i].name == prop_name)
			return cl_def_prop[i];
	//	if not found, return  an empty DbDatum
	return Tango::DbDatum(prop_name);
}


//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::set_default_property()
 *	Description : Set default property (class and device) for wizard.
 *                For each property, add to wizard property name and description.
 *                If default value has been set, add it to wizard property and
 *                store it in a DbDatum.
 */
//--------------------------------------------------------
void JaiGenicamDSClass::set_default_property()
{
	string	prop_name;
	string	prop_desc;
	string	prop_def;
	vector<string>	vect_data;

	//	Set Default Class Properties

	//	Set Default device Properties
	prop_name = "id_number";
	prop_desc = "Id of the camera, used to identify it on the network. \nType ip address or serial number of the camera.";
	prop_def  = "";
	vect_data.clear();
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "gain_node_name";
	prop_desc = "Name of gain node in the camera genicam node tree.\nNormally ``Gain`` or ``GainRaw``.";
	prop_def  = "";
	vect_data.clear();
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "exposuretime_node_name";
	prop_desc = "Name of exposuretime node in camera genicam node tree. \nNormally ``ExposureTime`` or ``ExposureTimeAbs``";
	prop_def  = "";
	vect_data.clear();
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "triggersource_node_name";
	prop_desc = "Name of trigger source node in camera genicam node tree. \nNormally ``TriggerSource`";
	prop_def  = "";
	vect_data.clear();
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "triggermode_node_name";
	prop_desc = "Name of trigger mode node in camera genicam node tree. \nNormally ``TriggerMode``";
	prop_def  = "TriggerMode";
	vect_data.clear();
	vect_data.push_back("TriggerMode");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "width_node_name";
	prop_desc = "Name of image width node in camera genicam node tree. \nNormally ``Width``";
	prop_def  = "Width";
	vect_data.clear();
	vect_data.push_back("Width");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "height_node_name";
	prop_desc = "Name of image height node in camera genicam node tree. \nNormally ``Height``";
	prop_def  = "Height";
	vect_data.clear();
	vect_data.push_back("Height");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "offsetx_node_name";
	prop_desc = "Name of image offset x node in camera genicam node tree. \nNormally ``OffsetX``";
	prop_def  = "OffsetX";
	vect_data.clear();
	vect_data.push_back("OffsetX");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "offsety_node_name";
	prop_desc = "Name of image offset y node in camera genicam node tree. \nNormally ``OffsetY``";
	prop_def  = "OffsetY";
	vect_data.clear();
	vect_data.push_back("OffsetY");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
	prop_name = "pixelformat_node_name";
	prop_desc = "Name of pixelformat node in camera genicam node tree. \nNormally ``PixelFormat``";
	prop_def  = "PixelFormat";
	vect_data.clear();
	vect_data.push_back("PixelFormat");
	if (prop_def.length()>0)
	{
		Tango::DbDatum	data(prop_name);
		data << vect_data ;
		dev_def_prop.push_back(data);
		add_wiz_dev_prop(prop_name, prop_desc,  prop_def);
	}
	else
		add_wiz_dev_prop(prop_name, prop_desc);
}

//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::write_class_property()
 *	Description : Set class description fields as property in database
 */
//--------------------------------------------------------
void JaiGenicamDSClass::write_class_property()
{
	//	First time, check if database used
	if (Tango::Util::_UseDb == false)
		return;

	Tango::DbData	data;
	string	classname = get_name();
	string	header;
	string::size_type	start, end;

	//	Put title
	Tango::DbDatum	title("ProjectTitle");
	string	str_title("JaiGenicamDS");
	title << str_title;
	data.push_back(title);

	//	Put Description
	Tango::DbDatum	description("Description");
	vector<string>	str_desc;
	str_desc.push_back("Control of Jai Genicam cameras");
	description << str_desc;
	data.push_back(description);

	//	put cvs or svn location
	string	filename("JaiGenicamDS");
	filename += "Class.cpp";

	// check for cvs information
	string	src_path(CvsPath);
	start = src_path.find("/");
	if (start!=string::npos)
	{
		end   = src_path.find(filename);
		if (end>start)
		{
			string	strloc = src_path.substr(start, end-start);
			//	Check if specific repository
			start = strloc.find("/cvsroot/");
			if (start!=string::npos && start>0)
			{
				string	repository = strloc.substr(0, start);
				if (repository.find("/segfs/")!=string::npos)
					strloc = "ESRF:" + strloc.substr(start, strloc.length()-start);
			}
			Tango::DbDatum	cvs_loc("cvs_location");
			cvs_loc << strloc;
			data.push_back(cvs_loc);
		}
	}

	// check for svn information
	else
	{
		string	src_path(SvnPath);
		start = src_path.find("://");
		if (start!=string::npos)
		{
			end = src_path.find(filename);
			if (end>start)
			{
				header = "$HeadURL: ";
				start = header.length();
				string	strloc = src_path.substr(start, (end-start));
				
				Tango::DbDatum	svn_loc("svn_location");
				svn_loc << strloc;
				data.push_back(svn_loc);
			}
		}
	}

	//	Get CVS or SVN revision tag
	
	// CVS tag
	string	tagname(TagName);
	header = "$Name: ";
	start = header.length();
	string	endstr(" $");
	
	end   = tagname.find(endstr);
	if (end!=string::npos && end>start)
	{
		string	strtag = tagname.substr(start, end-start);
		Tango::DbDatum	cvs_tag("cvs_tag");
		cvs_tag << strtag;
		data.push_back(cvs_tag);
	}
	
	// SVN tag
	string	svnpath(SvnPath);
	header = "$HeadURL: ";
	start = header.length();
	
	end   = svnpath.find(endstr);
	if (end!=string::npos && end>start)
	{
		string	strloc = svnpath.substr(start, end-start);
		
		string tagstr ("/tags/");
		start = strloc.find(tagstr);
		if ( start!=string::npos )
		{
			start = start + tagstr.length();
			end   = strloc.find(filename);
			string	strtag = strloc.substr(start, end-start-1);
			
			Tango::DbDatum	svn_tag("svn_tag");
			svn_tag << strtag;
			data.push_back(svn_tag);
		}
	}

	//	Get URL location
	string	httpServ(HttpServer);
	if (httpServ.length()>0)
	{
		Tango::DbDatum	db_doc_url("doc_url");
		db_doc_url << httpServ;
		data.push_back(db_doc_url);
	}

	//  Put inheritance
	Tango::DbDatum	inher_datum("InheritedFrom");
	vector<string> inheritance;
	inheritance.push_back("TANGO_BASE_CLASS");
	inher_datum << inheritance;
	data.push_back(inher_datum);

	//	Call database and and values
	get_db_class()->put_property(data);
}

//===================================================================
//	Factory methods
//===================================================================

//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::device_factory()
 *	Description : Create the device object(s)
 *                and store them in the device list
 */
//--------------------------------------------------------
void JaiGenicamDSClass::device_factory(const Tango::DevVarStringArray *devlist_ptr)
{
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::device_factory_before) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::device_factory_before

	//	Create devices and add it into the device list
	for (unsigned long i=0 ; i<devlist_ptr->length() ; i++)
	{
		cout4 << "Device name : " << (*devlist_ptr)[i].in() << endl;
		device_list.push_back(new JaiGenicamDS(this, (*devlist_ptr)[i]));							 
	}

	//	Manage dynamic attributes if any
	erase_dynamic_attributes(devlist_ptr, get_class_attr()->get_attr_list());

	//	Export devices to the outside world
	for (unsigned long i=1 ; i<=devlist_ptr->length() ; i++)
	{
		//	Add dynamic attributes if any
		JaiGenicamDS *dev = static_cast<JaiGenicamDS *>(device_list[device_list.size()-i]);
		dev->add_dynamic_attributes();

		//	Check before if database used.
		if ((Tango::Util::_UseDb == true) && (Tango::Util::_FileDb == false))
			export_device(dev);
		else
			export_device(dev, dev->get_name().c_str());
	}

	/*----- PROTECTED REGION ID(JaiGenicamDSClass::device_factory_after) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::device_factory_after
}
//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::attribute_factory()
 *	Description : Create the attribute object(s)
 *                and store them in the attribute list
 */
//--------------------------------------------------------
void JaiGenicamDSClass::attribute_factory(vector<Tango::Attr *> &att_list)
{
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::attribute_factory_before) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::attribute_factory_before
	//	Attribute : ExposureTime
	ExposureTimeAttrib	*exposuretime = new ExposureTimeAttrib();
	Tango::UserDefaultAttrProp	exposuretime_prop;
	exposuretime_prop.set_description("Exposure time of the capture in milliseconds");
	exposuretime_prop.set_label("Exposure time");
	exposuretime_prop.set_unit("ms");
	exposuretime_prop.set_standard_unit("ms");
	exposuretime_prop.set_display_unit("ms");
	//	format	not set for ExposureTime
	//	max_value	not set for ExposureTime
	exposuretime_prop.set_min_value("0");
	//	max_alarm	not set for ExposureTime
	//	min_alarm	not set for ExposureTime
	//	max_warning	not set for ExposureTime
	//	min_warning	not set for ExposureTime
	//	delta_t	not set for ExposureTime
	//	delta_val	not set for ExposureTime
	
	exposuretime->set_default_properties(exposuretime_prop);
	//	Not Polled
	exposuretime->set_disp_level(Tango::OPERATOR);
	exposuretime->set_memorized();
	exposuretime->set_memorized_init(true);
	att_list.push_back(exposuretime);

	//	Attribute : Gain
	GainAttrib	*gain = new GainAttrib();
	Tango::UserDefaultAttrProp	gain_prop;
	gain_prop.set_description("Camera gain value. The unit depends on the specific camera implementation.");
	gain_prop.set_label("gain");
	//	unit	not set for Gain
	//	standard_unit	not set for Gain
	//	display_unit	not set for Gain
	//	format	not set for Gain
	//	max_value	not set for Gain
	gain_prop.set_min_value("0");
	//	max_alarm	not set for Gain
	//	min_alarm	not set for Gain
	//	max_warning	not set for Gain
	//	min_warning	not set for Gain
	//	delta_t	not set for Gain
	//	delta_val	not set for Gain
	
	gain->set_default_properties(gain_prop);
	//	Not Polled
	gain->set_disp_level(Tango::OPERATOR);
	gain->set_memorized();
	gain->set_memorized_init(true);
	att_list.push_back(gain);

	//	Attribute : FrameRate
	FrameRateAttrib	*framerate = new FrameRateAttrib();
	Tango::UserDefaultAttrProp	framerate_prop;
	framerate_prop.set_description("Frame rate of the camera when free running");
	framerate_prop.set_label("frame rate");
	framerate_prop.set_unit("Hz");
	framerate_prop.set_standard_unit("Hz");
	framerate_prop.set_display_unit("Hz");
	//	format	not set for FrameRate
	//	max_value	not set for FrameRate
	framerate_prop.set_min_value("0");
	//	max_alarm	not set for FrameRate
	//	min_alarm	not set for FrameRate
	//	max_warning	not set for FrameRate
	//	min_warning	not set for FrameRate
	//	delta_t	not set for FrameRate
	//	delta_val	not set for FrameRate
	
	framerate->set_default_properties(framerate_prop);
	//	Not Polled
	framerate->set_disp_level(Tango::OPERATOR);
	//	Not Memorized
	att_list.push_back(framerate);

	//	Attribute : TriggerSource
	TriggerSourceAttrib	*triggersource = new TriggerSourceAttrib();
	Tango::UserDefaultAttrProp	triggersource_prop;
	triggersource_prop.set_description("Select if external (true) or internal (false) triggering is to be used.");
	triggersource_prop.set_label("external trigger");
	//	unit	not set for TriggerSource
	//	standard_unit	not set for TriggerSource
	//	display_unit	not set for TriggerSource
	//	format	not set for TriggerSource
	//	max_value	not set for TriggerSource
	//	min_value	not set for TriggerSource
	//	max_alarm	not set for TriggerSource
	//	min_alarm	not set for TriggerSource
	//	max_warning	not set for TriggerSource
	//	min_warning	not set for TriggerSource
	//	delta_t	not set for TriggerSource
	//	delta_val	not set for TriggerSource
	
	triggersource->set_default_properties(triggersource_prop);
	//	Not Polled
	triggersource->set_disp_level(Tango::OPERATOR);
	triggersource->set_memorized();
	triggersource->set_memorized_init(true);
	att_list.push_back(triggersource);

	//	Attribute : FrameCounter
	FrameCounterAttrib	*framecounter = new FrameCounterAttrib();
	Tango::UserDefaultAttrProp	framecounter_prop;
	framecounter_prop.set_description("Counts number of frames captured since starting acquisition. ");
	framecounter_prop.set_label("frame counter");
	//	unit	not set for FrameCounter
	//	standard_unit	not set for FrameCounter
	//	display_unit	not set for FrameCounter
	//	format	not set for FrameCounter
	//	max_value	not set for FrameCounter
	//	min_value	not set for FrameCounter
	//	max_alarm	not set for FrameCounter
	//	min_alarm	not set for FrameCounter
	//	max_warning	not set for FrameCounter
	//	min_warning	not set for FrameCounter
	//	delta_t	not set for FrameCounter
	//	delta_val	not set for FrameCounter
	
	framecounter->set_default_properties(framecounter_prop);
	//	Not Polled
	framecounter->set_disp_level(Tango::OPERATOR);
	//	Not Memorized
	att_list.push_back(framecounter);

	//	Attribute : TriggerMode
	TriggerModeAttrib	*triggermode = new TriggerModeAttrib();
	Tango::UserDefaultAttrProp	triggermode_prop;
	//	description	not set for TriggerMode
	//	label	not set for TriggerMode
	//	unit	not set for TriggerMode
	//	standard_unit	not set for TriggerMode
	//	display_unit	not set for TriggerMode
	//	format	not set for TriggerMode
	//	max_value	not set for TriggerMode
	//	min_value	not set for TriggerMode
	//	max_alarm	not set for TriggerMode
	//	min_alarm	not set for TriggerMode
	//	max_warning	not set for TriggerMode
	//	min_warning	not set for TriggerMode
	//	delta_t	not set for TriggerMode
	//	delta_val	not set for TriggerMode
	
	triggermode->set_default_properties(triggermode_prop);
	//	Not Polled
	triggermode->set_disp_level(Tango::OPERATOR);
	triggermode->set_memorized();
	triggermode->set_memorized_init(true);
	att_list.push_back(triggermode);

	//	Attribute : ImageHeight
	ImageHeightAttrib	*imageheight = new ImageHeightAttrib();
	Tango::UserDefaultAttrProp	imageheight_prop;
	//	description	not set for ImageHeight
	//	label	not set for ImageHeight
	//	unit	not set for ImageHeight
	//	standard_unit	not set for ImageHeight
	//	display_unit	not set for ImageHeight
	//	format	not set for ImageHeight
	//	max_value	not set for ImageHeight
	//	min_value	not set for ImageHeight
	//	max_alarm	not set for ImageHeight
	//	min_alarm	not set for ImageHeight
	//	max_warning	not set for ImageHeight
	//	min_warning	not set for ImageHeight
	//	delta_t	not set for ImageHeight
	//	delta_val	not set for ImageHeight
	
	imageheight->set_default_properties(imageheight_prop);
	//	Not Polled
	imageheight->set_disp_level(Tango::OPERATOR);
	imageheight->set_memorized();
	imageheight->set_memorized_init(true);
	att_list.push_back(imageheight);

	//	Attribute : ImageWidth
	ImageWidthAttrib	*imagewidth = new ImageWidthAttrib();
	Tango::UserDefaultAttrProp	imagewidth_prop;
	//	description	not set for ImageWidth
	//	label	not set for ImageWidth
	//	unit	not set for ImageWidth
	//	standard_unit	not set for ImageWidth
	//	display_unit	not set for ImageWidth
	//	format	not set for ImageWidth
	//	max_value	not set for ImageWidth
	//	min_value	not set for ImageWidth
	//	max_alarm	not set for ImageWidth
	//	min_alarm	not set for ImageWidth
	//	max_warning	not set for ImageWidth
	//	min_warning	not set for ImageWidth
	//	delta_t	not set for ImageWidth
	//	delta_val	not set for ImageWidth
	
	imagewidth->set_default_properties(imagewidth_prop);
	//	Not Polled
	imagewidth->set_disp_level(Tango::OPERATOR);
	imagewidth->set_memorized();
	imagewidth->set_memorized_init(true);
	att_list.push_back(imagewidth);

	//	Attribute : ImageOffsetX
	ImageOffsetXAttrib	*imageoffsetx = new ImageOffsetXAttrib();
	Tango::UserDefaultAttrProp	imageoffsetx_prop;
	//	description	not set for ImageOffsetX
	//	label	not set for ImageOffsetX
	//	unit	not set for ImageOffsetX
	//	standard_unit	not set for ImageOffsetX
	//	display_unit	not set for ImageOffsetX
	//	format	not set for ImageOffsetX
	//	max_value	not set for ImageOffsetX
	//	min_value	not set for ImageOffsetX
	//	max_alarm	not set for ImageOffsetX
	//	min_alarm	not set for ImageOffsetX
	//	max_warning	not set for ImageOffsetX
	//	min_warning	not set for ImageOffsetX
	//	delta_t	not set for ImageOffsetX
	//	delta_val	not set for ImageOffsetX
	
	imageoffsetx->set_default_properties(imageoffsetx_prop);
	//	Not Polled
	imageoffsetx->set_disp_level(Tango::OPERATOR);
	imageoffsetx->set_memorized();
	imageoffsetx->set_memorized_init(true);
	att_list.push_back(imageoffsetx);

	//	Attribute : ImageOffsetY
	ImageOffsetYAttrib	*imageoffsety = new ImageOffsetYAttrib();
	Tango::UserDefaultAttrProp	imageoffsety_prop;
	//	description	not set for ImageOffsetY
	//	label	not set for ImageOffsetY
	//	unit	not set for ImageOffsetY
	//	standard_unit	not set for ImageOffsetY
	//	display_unit	not set for ImageOffsetY
	//	format	not set for ImageOffsetY
	//	max_value	not set for ImageOffsetY
	//	min_value	not set for ImageOffsetY
	//	max_alarm	not set for ImageOffsetY
	//	min_alarm	not set for ImageOffsetY
	//	max_warning	not set for ImageOffsetY
	//	min_warning	not set for ImageOffsetY
	//	delta_t	not set for ImageOffsetY
	//	delta_val	not set for ImageOffsetY
	
	imageoffsety->set_default_properties(imageoffsety_prop);
	//	Not Polled
	imageoffsety->set_disp_level(Tango::OPERATOR);
	imageoffsety->set_memorized();
	imageoffsety->set_memorized_init(true);
	att_list.push_back(imageoffsety);

	//	Attribute : PixelFormat
	PixelFormatAttrib	*pixelformat = new PixelFormatAttrib();
	Tango::UserDefaultAttrProp	pixelformat_prop;
	//	description	not set for PixelFormat
	//	label	not set for PixelFormat
	//	unit	not set for PixelFormat
	//	standard_unit	not set for PixelFormat
	//	display_unit	not set for PixelFormat
	//	format	not set for PixelFormat
	//	max_value	not set for PixelFormat
	//	min_value	not set for PixelFormat
	//	max_alarm	not set for PixelFormat
	//	min_alarm	not set for PixelFormat
	//	max_warning	not set for PixelFormat
	//	min_warning	not set for PixelFormat
	//	delta_t	not set for PixelFormat
	//	delta_val	not set for PixelFormat
	
	pixelformat->set_default_properties(pixelformat_prop);
	//	Not Polled
	pixelformat->set_disp_level(Tango::OPERATOR);
	pixelformat->set_memorized();
	pixelformat->set_memorized_init(true);
	att_list.push_back(pixelformat);

	//	Attribute : Image
	ImageAttrib	*image = new ImageAttrib();
	Tango::UserDefaultAttrProp	image_prop;
	image_prop.set_description("Latest captured image");
	image_prop.set_label("image");
	//	unit	not set for Image
	//	standard_unit	not set for Image
	//	display_unit	not set for Image
	//	format	not set for Image
	//	max_value	not set for Image
	//	min_value	not set for Image
	//	max_alarm	not set for Image
	//	min_alarm	not set for Image
	//	max_warning	not set for Image
	//	min_warning	not set for Image
	//	delta_t	not set for Image
	//	delta_val	not set for Image
	
	image->set_default_properties(image_prop);
	//	Not Polled
	image->set_disp_level(Tango::OPERATOR);
	//	Not Memorized
	att_list.push_back(image);

	//	Create a list of static attributes
	create_static_attribute_list(get_class_attr()->get_attr_list());
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::attribute_factory_after) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::attribute_factory_after
}
//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::command_factory()
 *	Description : Create the command object(s)
 *                and store them in the command list
 */
//--------------------------------------------------------
void JaiGenicamDSClass::command_factory()
{
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::command_factory_before) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::command_factory_before


	//	Command start
	startClass	*pstartCmd =
		new startClass("start",
			Tango::DEV_VOID, Tango::DEV_VOID,
			"",
			"",
			Tango::OPERATOR);
	command_list.push_back(pstartCmd);

	//	Command stop
	stopClass	*pstopCmd =
		new stopClass("stop",
			Tango::DEV_VOID, Tango::DEV_VOID,
			"",
			"",
			Tango::OPERATOR);
	command_list.push_back(pstopCmd);

	//	Command Off
	OffClass	*pOffCmd =
		new OffClass("Off",
			Tango::DEV_VOID, Tango::DEV_VOID,
			"",
			"",
			Tango::OPERATOR);
	command_list.push_back(pOffCmd);

	//	Command On
	OnClass	*pOnCmd =
		new OnClass("On",
			Tango::DEV_VOID, Tango::DEV_VOID,
			"",
			"",
			Tango::OPERATOR);
	command_list.push_back(pOnCmd);

	//	Command GetCameraList
	GetCameraListClass	*pGetCameraListCmd =
		new GetCameraListClass("GetCameraList",
			Tango::DEV_VOID, Tango::DEVVAR_STRINGARRAY,
			"",
			"",
			Tango::OPERATOR);
	command_list.push_back(pGetCameraListCmd);

	/*----- PROTECTED REGION ID(JaiGenicamDSClass::command_factory_after) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::command_factory_after
}

//===================================================================
//	Dynamic attributes related methods
//===================================================================

//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::create_static_attribute_list
 * description : 	Create the a list of static attributes
 *
 * @param	att_list	the ceated attribute list 
 */
//--------------------------------------------------------
void JaiGenicamDSClass::create_static_attribute_list(vector<Tango::Attr *> &att_list)
{
	for (unsigned long i=0 ; i<att_list.size() ; i++)
	{
		string att_name(att_list[i]->get_name());
		transform(att_name.begin(), att_name.end(), att_name.begin(), ::tolower);
		defaultAttList.push_back(att_name);
	}

	cout2 << defaultAttList.size() << " attributes in default list" << endl;

	/*----- PROTECTED REGION ID(JaiGenicamDSClass::create_static_att_list) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::create_static_att_list
}


//--------------------------------------------------------
/**
 * method : 		JaiGenicamDSClass::erase_dynamic_attributes
 * description : 	delete the dynamic attributes if any.
 *
 * @param	devlist_ptr	the device list pointer
 * @param	list of all attributes
 */
//--------------------------------------------------------
void JaiGenicamDSClass::erase_dynamic_attributes(const Tango::DevVarStringArray *devlist_ptr, vector<Tango::Attr *> &att_list)
{
	Tango::Util *tg = Tango::Util::instance();

	for (unsigned long i=0 ; i<devlist_ptr->length() ; i++)
	{	
		Tango::DeviceImpl *dev_impl = tg->get_device_by_name(((string)(*devlist_ptr)[i]).c_str());
		JaiGenicamDS *dev = static_cast<JaiGenicamDS *> (dev_impl);
		
		vector<Tango::Attribute *> &dev_att_list = dev->get_device_attr()->get_attribute_list();
		vector<Tango::Attribute *>::iterator ite_att;
		for (ite_att=dev_att_list.begin() ; ite_att != dev_att_list.end() ; ++ite_att)
		{
			string att_name((*ite_att)->get_name_lower());
			if ((att_name == "state") || (att_name == "status"))
				continue;
			vector<string>::iterator ite_str = find(defaultAttList.begin(), defaultAttList.end(), att_name);
			if (ite_str == defaultAttList.end())
			{
				cout2 << att_name << " is a UNWANTED dynamic attribute for device " << (*devlist_ptr)[i] << endl;
				Tango::Attribute &att = dev->get_device_attr()->get_attr_by_name(att_name.c_str());
				dev->remove_attribute(att_list[att.get_attr_idx()], true, false);
				--ite_att;
			}
		}
	}
	/*----- PROTECTED REGION ID(JaiGenicamDSClass::erase_dynamic_attributes) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::erase_dynamic_attributes
}

//--------------------------------------------------------
/**
 *	Method      : JaiGenicamDSClass::get_attr_by_name()
 *	Description : returns Tango::Attr * object found by name
 */
//--------------------------------------------------------
Tango::Attr *JaiGenicamDSClass::get_attr_object_by_name(vector<Tango::Attr *> &att_list, string attname)
{
	vector<Tango::Attr *>::iterator it;
	for (it=att_list.begin() ; it<att_list.end() ; it++)
		if ((*it)->get_name()==attname)
			return (*it);
	//	Attr does not exist
	return NULL;
}


/*----- PROTECTED REGION ID(JaiGenicamDSClass::Additional Methods) ENABLED START -----*/

/*----- PROTECTED REGION END -----*/	//	JaiGenicamDSClass::Additional Methods
} //	namespace
