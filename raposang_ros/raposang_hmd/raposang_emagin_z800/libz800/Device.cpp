/* C89 */
#include <cstdio>
#include <cstring>

/* STL */
#include <iostream>
#include <algorithm>

/* libusb */
#include <libusb.h>

/* Local */
#include "Device.h"

enum {
	VERB_PEEK_EEPROM      = 0x81,
	VERB_SET_POKE_ADDR    = 0x82,
	VERB_POKE_EEPROM      = 0x83,
	VERB_RESET            = 0x84,
	VERB_SLEEP            = 0x85,
	VERB_PEEK_OLED_0      = 0x86,
	VERB_PEEK_OLED_1      = 0x87,
	VERB_STEP_BRIGHTNESS  = 0x88,
	VERB_QUERY_STATE      = 0x89,
	VERB_WAKE             = 0x8a,
	VERB_3DMODE           = 0x8b,
	VERB_3DFLIP           = 0x8c
};

Device::Device(int debugLevel)
: _ctx(NULL)
, _tracker(NULL)
, _display(NULL)
, _debugLevel(debugLevel)
{
	std::vector<Property> properties = Property::getProperties();
	for(std::vector<Property>::const_iterator it = properties.begin();
		it != properties.end(); ++it)
	{
		_properties[it->getName()] = *it;

		char *name = new char [it->getName().size() + 1];
		strcpy(name, it->getName().c_str());
		const_cast<std::vector<char *> &>(_propertyNames).push_back(name);
	}

	const_cast<std::vector<char *> &>(_propertyNames).push_back(NULL);
}

Device::~Device()
{
	disconnect();

	for(std::vector<char *>::const_iterator it = _propertyNames.begin();
		it != _propertyNames.end(); ++it)
	{
		delete [] *it;
	}
}

const char * const *Device::getPropertyNames()
{
	return &_propertyNames[0];
}

bool Device::connect()
{
	if(_debugLevel > 1)
	{
		std::cerr << "<libz800> libusb_init(&_ctx)" << std::endl;
	}
	
	int r = libusb_init(&_ctx);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_init() failed: " << r << std::endl;
		return false;
	}

	if(_debugLevel > 2)
	{
		libusb_set_debug(_ctx, _debugLevel - 2);
	}
	else
	{
		libusb_set_debug(_ctx, 0);
	}

#ifdef WIN32
	Sleep(50);
#endif

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_open_device_with_vid_pid(_ctx, 0x1641, 0x0110)" << std::endl;
	_tracker = libusb_open_device_with_vid_pid(_ctx, 0x1641, 0x0110);
	if(_tracker == NULL)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> Failed to find tracker device 1641:0110" << std::endl;
		disconnect();
		return false;
	}

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_open_device_with_vid_pid(_ctx, 0x1641, 0x0120)" << std::endl;
	_display = libusb_open_device_with_vid_pid(_ctx, 0x1641, 0x0120);
	if(_display == NULL)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> Failed to find display device 1641:0120" << std::endl;
		disconnect();
		return false;
	}

#ifdef WIN32
	Sleep(50);
#endif

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_set_configuration(_tracker, 1)" << std::endl;
	r = libusb_set_configuration(_tracker, 1);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_set_configuration() failed for tracker: " << r << std::endl;
		disconnect();
		return false;
	}

#ifdef WIN32
	Sleep(50);
#endif

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_claim_interface(_tracker, 0)" << std::endl;
	r = libusb_claim_interface(_tracker, 0);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_claim_interface() failed for tracker: " << r << std::endl;
		disconnect();
		return false;
	}

#ifndef WIN32
	if(_debugLevel > 1) std::cerr << "<libz800> libusb_kernel_driver_active(_display, 0)" << std::endl;
	r = libusb_kernel_driver_active(_display, 0);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_kernel_driver_active() failed for display: " << r << std::endl;
		/* We can still try to claim the interface so let's go on. */
	}
	else if (r == 1)
	{
		if(_debugLevel > 1) std::cerr << "<libz800> libusb_kernel_driver_active() reported 1" << std::endl;
		if(_debugLevel > 1) std::cerr << "<libz800> libusb_kernel_driver_active(_display, 0)" << std::endl;
		r = libusb_detach_kernel_driver(_display, 0);
		if(r < 0)
		{
			if(_debugLevel > 0) std::cerr << "<libz800> libusb_detach_kernel_driver() failed: " << r << std::endl;
			disconnect();
			return false;
		}
	}
#endif


#ifdef WIN32
	Sleep(50);
#endif

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_set_configuration(_display, 1)" << std::endl;
	r = libusb_set_configuration(_display, 1);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_set_configuration() failed for display: " << r << std::endl;
		disconnect();
		return false;
	}

#ifdef WIN32
	Sleep(50);
#endif

	if(_debugLevel > 1) std::cerr << "<libz800> libusb_claim_interface(_display, 0)" << std::endl;
	r = libusb_claim_interface(_display, 0);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "<libz800> libusb_claim_interface() failed for display: " << r << std::endl;
		disconnect();
		return false;
	}

#ifdef WIN32
	Sleep(50);
#endif

	/**
	 * Try to wake the device.
	 */
	if(_debugLevel > 1) std::cerr << "<libz800> waking the device" << std::endl;
	if(!wake())
	{
		disconnect();
		return false;
	}

	/**
	 * Enable the display.
	 */
	if(_debugLevel > 1) std::cerr << "<libz800> enabling display" << std::endl;
	if(!setStereoMode(false))
	{
		disconnect();
		return false;
	}

	if(_debugLevel > 1) std::cerr << "<libz800> successfully connected to z800 device" << std::endl;

	return true;
}

void Device::disconnect()
{
	if(_debugLevel > 1) std::cerr << "<libz800> disconnecting..." << std::endl; 

	if(_display)
	{
		if(_debugLevel > 1) std::cerr << "<libz800> closing display" << std::endl; 
		libusb_close(_display);
		_display = NULL;
	}
	if(_tracker)
	{
		if(_debugLevel > 1) std::cerr << "<libz800> closing tracker" << std::endl; 
		libusb_close(_tracker);
		_tracker = NULL;
	}
	if(_ctx)
	{
		if(_debugLevel > 1) std::cerr << "<libz800> calling libusb_exit()" << std::endl; 
		libusb_exit(_ctx);
		_ctx = NULL;
	}
	if(_debugLevel > 1) std::cerr << "<libz800> disconnected" << std::endl; 
}

void Device::setDebugLevel(int debugLevel)
{
	_debugLevel = debugLevel;

	if(_debugLevel > 2)
	{
		libusb_set_debug(_ctx, _debugLevel - 2);
	}
	else
	{
		libusb_set_debug(_ctx, 0);
	}
}

int Device::getFirmwareVersion()
{
	int version;

	if(readProperty("version", version, false) == false)
	{
		return -1;
	}

	return version;
}

bool Device::pollDisplay()
{
	if(!_display) return false;

	unsigned char buf[64];
	std::fill(buf, buf + sizeof(buf), 0);

	int nread = 0;

	int r = libusb_interrupt_transfer(_display, 0x81, buf, sizeof(buf), &nread, 100000);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "libusb_interrupt_transfer() failed to read: " << r << std::endl;
		return false;
	}

#if 0
	printf("---\n");
	for(unsigned i = 0; i < nread; ++i)
	{
		printf("%.2x: %.2x\n", i, buf[i]);
	}
#endif

	_eepromRead = buf[4];
	_lastCmd = buf[2];

	return true;
}

bool Device::sendCmdRaw(int verb, int noun)
{
	if(!_display) return false;

	unsigned char buf[64];
	std::fill(buf, buf + sizeof(buf), 0);

	buf[0] = verb;
	buf[1] = noun;

	int nwrote = 0;

	int r = libusb_interrupt_transfer(_display, 0x02, buf, 5, &nwrote, 10000);
	if(r < 0)
	{
		if(_debugLevel > 0) std::cerr << "libusb_interrupt_transfer() failed to send " << verb << "," << noun << " failed: " << r << std::endl;
		return false;
	}

	return true;
}

bool Device::sendCmd(int verb, int noun = 0)
{
	/**
	 * (Passive) Verb used to handshake this command.
	 */
	int syncVerb = (verb == VERB_PEEK_EEPROM ? VERB_QUERY_STATE : VERB_PEEK_EEPROM);

	/**
	 * Fill the last-command buffer with the synchronization verb.
	 */
	do {
		if(!sendCmdRaw(syncVerb, 0) || !pollDisplay()) return false;
	} while(_lastCmd != syncVerb);

	/**
	 * Issue command until confirmed.
	 */
	do {
		if(!sendCmdRaw(verb, noun) || !pollDisplay()) return false;
	} while(_lastCmd != verb);

	return true;
}

Property *Device::getProperty(const std::string &name)
{
	if(_properties.count(name) == 0) return NULL;
	return &_properties[name];
}

bool Device::readProperty(const std::string &name, int &value, bool rightDisplay)
{
	if(_properties.count(name) == 0)
	{
		return false;
	}

	const Property &prop = _properties[name];

	if(rightDisplay)
	{
		if(prop.getRightLowAddr() < 0) return false;

		if(sendCmd(VERB_PEEK_EEPROM, prop.getRightLowAddr()) == false) return false;
		value = _eepromRead;

		if(prop.getRightHighAddr() > -1)
		{
			if(sendCmd(VERB_PEEK_EEPROM, prop.getRightHighAddr()) == false) return false;
			value = (_eepromRead << 8) | value;
		}
	}
	else
	{
		if(prop.getLeftLowAddr() < 0) return false;

		if(sendCmd(VERB_PEEK_EEPROM, prop.getLeftLowAddr()) == false) return false;
		value = _eepromRead;

		if(prop.getLeftHighAddr() > -1)
		{
			if(sendCmd(VERB_PEEK_EEPROM, prop.getLeftHighAddr()) == false) return false;
			value = (_eepromRead << 8) | value;
		}
	}

	return true;
}

bool Device::writeProperty(const std::string & /* name */, int /* value */, bool /* rightDisplay */)
{
	/* XXX: Implement */
	return false;
}

bool Device::readEEPROM(int address, int &value)
{
	if(sendCmd(VERB_PEEK_EEPROM, address) == false) return false;
	value = _eepromRead;
	return true;
}

bool Device::writeEEPROM(int /* address */, int /* value */)
{
	return false;
}

std::vector<std::string> Device::getProperties()
{
	std::vector<std::string> ret;

	for(std::map<std::string, Property>::const_iterator it = _properties.begin();
		it != _properties.end(); ++it)
	{
		ret.push_back(it->first);
	}

	return ret;
}

bool Device::wake()
{
	return sendCmd(VERB_WAKE, 0);
}

bool Device::stepBrightness()
{
	return sendCmd(VERB_STEP_BRIGHTNESS, 0);
}

bool Device::setStereoMode(bool enabled)
{
	return sendCmd(VERB_3DMODE, enabled);
}

bool Device::flipStereo()
{
	return sendCmd(VERB_3DFLIP, 0);
}

double Device::getTrackerData(Vec3f &gyr, Vec3f &acc, Vec3f &mag)
{
	if(!_tracker) return false;

	bool gotMag = false;
	bool gotGyrAcc = false;
	float dt = 0.0;

	for(int i = 0; i < 2; ++i)
	{
		unsigned char buf[8];
		int nread = 0;
		int r = libusb_interrupt_transfer(_tracker, 0x81, buf, sizeof(buf), &nread, 1000);
		if(r < 0)
		{
			if(_debugLevel > 0) std::cerr << "libusb_interrupt_transfer() failed to read: " << r << std::endl;
			return false;
		}

		if(buf[7] & 0xf0)
		{
			int my = ((int)(buf[7] & 0xf) << 6) | (buf[6] >> 2);
			int mz = ((int)(buf[6] & 0x3) << 8) | (buf[5]);
			int mx = ((int)buf[4] << 2) | (buf[3] >> 6);
			if(mx & 0x200) mx = -((mx ^ 0x3ff) + 1);
			if(my & 0x200) my = -((my ^ 0x3ff) + 1);
			if(mz & 0x200) mz = -((mz ^ 0x3ff) + 1);

			/* Normalize to [-1..1] */
			mag.x() = +(mx / 512.f);
			mag.y() = +(my / 512.f);
			mag.z() = -(mz / 512.f);

			gotMag = true;
		}
		else
		{
			dt = _timer.getElapsedTime();

			int ay = ((int)(buf[7] & 0xf)<<6) | (buf[6] >> 2);
			int ax = ((int)(buf[6] & 0x3)<<8) | (buf[5]);
			int az = ((int)buf[4] << 2) | (buf[3] >> 6);

			/* Normalize to [-1..1] */
			acc.x() = -(ax / 512.0 - 1.0);
			acc.y() = -(ay / 512.0 - 1.0);
			acc.z() = +(az / 512.0 - 1.0);

			int gy = ((int)(buf[3] & 0x3f) << 4) | (buf[2] >> 4);
			int gz = ((int)(buf[2] & 0xf) << 6) | (buf[1] >> 2);
			int gx = ((int)(buf[1] & 0x3) << 8) | (buf[0]);

			/* Normalize to [-1..1] */
			gyr.x() = -(gx / 512.0 - 1.0);
			gyr.y() = -(gy / 512.0 - 1.0);
			gyr.z() = -(gz / 512.0 - 1.0);

			gotGyrAcc = true;
		}
	}

	if(gotMag && gotGyrAcc)
	{
		return dt;
	}
	else
	{
		return -1.0;
	}
}

bool Device::dumpEEPROM()
{
	for(int i = 0; i < 256; ++i)
	{
		if(sendCmd(VERB_PEEK_EEPROM, i) == false)
		{
			std::cerr << "peek failed at " << i << std::endl;
		}

		std::string name = "(?)";
		for(std::map<std::string, Property>::iterator it = _properties.begin();
		    it != _properties.end(); ++it)
		{
			if(i == it->second.getLeftLowAddr()) name = it->second.getName() + " (" + (it->second.getRightLowAddr() > -1 ? "left, " : "") + "low)";
			if(i == it->second.getLeftHighAddr()) name = it->second.getName() + " (" + (it->second.getRightLowAddr() > -1 ? "left, " : "") + "high)";
			if(i == it->second.getRightLowAddr()) name = it->second.getName() + " (right, low)";
			if(i == it->second.getRightHighAddr()) name = it->second.getName() + " (right, high)";
		}

		printf("@ %.2x: %.2x  %s\n", i, _eepromRead, name.c_str());
	}

	return true;
}

