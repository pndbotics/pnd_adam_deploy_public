#include "vnIMU.h"

// yaw pitch roll gyro_x gyro_y gyro_z acc_x acc_y acc_z
Eigen::VectorXd vnIMU::imuData = Eigen::VectorXd::Zero(9);

vnIMU::~vnIMU(){
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
}
bool vnIMU::initIMU(){
	uint32_t oldBaud = 0;
	try{
		// Now let's create a VnSensor object and use it to connect to our sensor.
		vs.connect(SensorPort, SensorBaudrate);
		std::cout << "imu connect" << std::endl;
		//Change the baudrate
		oldBaud = vs.baudrate();
		std::cout << "imu baudrate" << std::endl;
		// try{
		vs.changeBaudRate(SensorBaudrate2);
		std::cout << "imu changeBaudRate" << std::endl;
		// vs.disconnect();
		// vs.connect(SensorPort, SensorBaudrate2);
	}catch(...){
		try{
			std::cout << "imu disconnect" << std::endl;
			vs.disconnect();
			std::cout << "imu retry connect" << std::endl;
			vs.connect(SensorPort, SensorBaudrate2);
		}catch(...){
			std::cout << "imu connect failed" << std::endl;
			return false;
		}
	}

	uint32_t newBaud = vs.baudrate();
	cout << "Old Baud Rate: " << oldBaud << " Hz" << endl;
	cout << "New Async Rate: " << newBaud << " Hz" << endl;
	//Close the AsyncDataOutput, set the frequency to 0
	uint32_t oldHz = vs.readAsyncDataOutputFrequency();
	vs.writeAsyncDataOutputFrequency(0);
	uint32_t newHz = vs.readAsyncDataOutputFrequency();
	cout << "Old Async Frequency: " << oldHz << " Hz" << endl;
	cout << "New Async Frequency: " << newHz << " Hz" << endl;
	// We change the heading mode used by the sensor.
	VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
	cout << "Old Heading Mode: " << vpeReg.headingMode << endl;
	vpeReg.headingMode = HEADINGMODE_RELATIVE;
	vs.writeVpeBasicControl(vpeReg, true);
	vpeReg = vs.readVpeBasicControl();
	cout << "New Heading Mode: " << vpeReg.headingMode << endl;
	
     // First we create a structure for setting the configuration information
	// for the binary output register to send yaw, pitch, roll data out at
	// 800 Hz.
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		1,
		COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
    GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
    GPSGROUP_NONE);
	std::cout<<"write binary output"<<std::endl;
    vs.writeBinaryOutput1(bor);
	std::cout<<"register"<<std::endl;
    vs.registerAsyncPacketReceivedHandler(NULL, vnIMU::asciiOrBinaryAsyncMessageReceived);

    return true;
}
bool vnIMU::closeIMU(){
    vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();

    return true;
}
void vnIMU::asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index){
	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
	{
		vec3f ypr;
		p.parseVNYPR(&ypr);
		cout << "ASCII Async YPR: " << ypr << endl;
		return;
	}

	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
	GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE,
	GPSGROUP_NONE))
	// Not the type of binary packet we are expecting.
			return;

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the Packet structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
		
		vec3f ypr = p.extractVec3f();
		vec3f dypr = p.extractVec3f();
		vec3f acc = p.extractVec3f();

		//trans ypr in NED to ypr in NWU
		imuData[0] = -ypr[0]/180.0*M_PI;//negative yaw
		imuData[1] = -ypr[1]/180.0*M_PI;//negative pitch
		imuData[2] = ypr[2] > 0.0 ? (ypr[2]/180.0 - 1.0)*M_PI : (ypr[2]/180.0 + 1.0)*M_PI;//roll + pi, modify to be in [-pi, pi]
		for (int i=0; i<3; ++i){
			imuData[i+3] = dypr[i];
			imuData[i+6] = acc[i];
		}

	}
}; 