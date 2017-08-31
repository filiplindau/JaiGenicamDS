#pragma once
#include <tango.h>
#include <Jai_Factory.h>
#include "JaiGenicamDS.h"
#include <mutex>
#include <thread>

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

class JaiGenicamState;

class JaiGenicamConnection
{
public:
	JaiGenicamConnection(JaiGenicamDS_ns::JaiGenicamDS* tango_ds_class, std::string camera_serial);
	~JaiGenicamConnection(void);

	/*void get_image();
	void get_gain();
	void set_gain();
	void get_exposuretime();
	void set_exposuretime();
	void get_framerate();
	void set_framerate();
	void get_externaltrigger();
	void set_externaltrigger();
	void get_framecounter();*/

	void start_thread();
	void stop_thread();
	void connect();
	void disconnect();

private:
	void run();

private:
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
	Tango::DevState tango_state;
	JaiGenicamDS_ns::JaiGenicamDS* tango_ds_class;

	std::mutex camera_mtx;

	FACTORY_HANDLE  factory_handle;     // Factory Handle
	CAM_HANDLE      camera_handle;      // Camera Handle
	std::string		camera_serial;
};


/////////////////////////////////////////////////////////////////////////////////

class JaiGenicamState 
{
public:	
	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );
	void close_camera(JaiGenicamConnection* wrapper);
	void close_factory(JaiGenicamConnection* wrapper);
	void connect(JaiGenicamConnection* wrapper);
	void disconnect(JaiGenicamConnection* wrapper);

protected:
	void change_state( JaiGenicamConnection*, JaiGenicamState* );
	
};

class JaiGenicamDisconnectedState : JaiGenicamState
{
public:
	static JaiGenicamState* Instance();

	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );
	virtual void connect( JaiGenicamConnection* wrapper );

};

class JaiGenicamInitState : JaiGenicamState
{
public:
	static JaiGenicamState* Instance();

	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );

};

class JaiGenicamConnectedState : JaiGenicamState
{
public:
	static JaiGenicamState* Instance();

	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );

};

class JaiGenicamFaultState : JaiGenicamState
{
public:
	static JaiGenicamState* Instance();

	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );

};

class JaiGenicamRunningState : JaiGenicamState
{
public:
	static JaiGenicamState* Instance();

	virtual void enter( JaiGenicamConnection* wrapper );
	virtual void run( JaiGenicamConnection* wrapper );

};