#include "Property.h"

std::vector<Property> Property::getProperties()
{
	std::vector<Property> ret;

	ret.push_back(Property("version",                 1,   0, -1, -1,  0, 0xffff, -1));

	ret.push_back(Property("brightness_0",            5,  -1,  2, -1,  32, 180,  80));
	ret.push_back(Property("brightness_1",            6,  -1,  3, -1,  32, 180,  64));
	ret.push_back(Property("brightness_2",            7,  -1,  4, -1,  32, 180,  48));

	ret.push_back(Property("saved_brightness_index",  8,  -1, -1, -1,   0,   6,   0));

	ret.push_back(Property("external_monitor",        9,  -1, -1, -1,   0,   1,   1));

	ret.push_back(Property("usb_timeout",            11,  10, -1, -1,  10, 30000,  300));
	ret.push_back(Property("ac_timeout",             13,  12, -1, -1,  10, 30000, 9000));

	ret.push_back(Property("stat",			128,  -1, 64, -1,  0, 0xff, -1));

	ret.push_back(Property("red_gain",              129,  -1, 65, -1,  64, 192, 128));
	ret.push_back(Property("red_gain_offset",       130,  -1, 66, -1,  32,  96,  48));

	ret.push_back(Property("green_gain",            131,  -1, 67, -1,  64, 192, 128));
	ret.push_back(Property("green_gain_offset",     132,  -1, 68, -1,  32,  96,  48));

	ret.push_back(Property("blue_gain",             133,  -1, 69, -1,  64, 192, 128));
	ret.push_back(Property("blue_gain_offset",      134,  -1, 70, -1,  32,  96,  48));

	ret.push_back(Property("mono_gain",             135,  -1, 71, -1,  0, 0xff, -1));
	ret.push_back(Property("mono_gain_offset",      136,  -1, 72, -1,  0, 0xff, -1));

	ret.push_back(Property("v_mode_svgaplus",       137,  -1, 73, -1,  0, 0xff, -1));
	ret.push_back(Property("h_mode_svgaplus",       138,  -1, 74, -1,  0, 0xff, -1));

	ret.push_back(Property("brightness",		139, 140, 75, 76,  0, 0xffff, -1));
	ret.push_back(Property("h_rate",		141, 142, 77, 78,  0, 0xffff, -1));

	ret.push_back(Property("pixelclock",            143, 144, 79, 80, 512, 6000, 9000));

	ret.push_back(Property("pif",			145,  -1, 81, -1,   0,  0xff, -1));

	ret.push_back(Property("phase",                 146,  -1, 82, -1,   0,  128, 0));

	ret.push_back(Property("h_start",               147,  -1, 83, -1, 117, 255, 186));
	ret.push_back(Property("v_start",               148,  -1, 84, -1,   5,  45,  25));

	ret.push_back(Property("hblk",			149,  -1, 85, -1,   0, 0xff,  -1));
	ret.push_back(Property("hdel",			150,  -1, 86, -1,   0, 0xff,  -1));
	ret.push_back(Property("pdwn",			151,  -1, 87, -1,   0, 0xff,  -1));
	ret.push_back(Property("atb",			152,  -1, 88, -1,   0, 0xff,  -1));
	ret.push_back(Property("amtest",		153,  -1, 89, -1,   0, 0xff,  -1));
	ret.push_back(Property("trim",			154,  -1, 90, -1,   0, 0xff,  -1));

	return ret;
}

Property::Property()
: _leftLowAddr(-1)
, _leftHighAddr(-1)
, _rightLowAddr(-1)
, _rightHighAddr(-1)
, _min(-1)
, _max(-1)
, _default(-1)
{
}


Property::Property(const std::string &name, int leftLowAddr, int leftHighAddr, int rightLowAddr, int rightHighAddr, int minValue, int maxValue, int defaultValue)
: _name(name)
, _leftLowAddr(leftLowAddr)
, _leftHighAddr(leftHighAddr)
, _rightLowAddr(rightLowAddr)
, _rightHighAddr(rightHighAddr)
, _min(minValue)
, _max(maxValue)
, _default(defaultValue)
{
}

Property::Property(const Property &rhs)
: _name(rhs._name)
, _leftLowAddr(rhs._leftLowAddr)
, _leftHighAddr(rhs._leftHighAddr)
, _rightLowAddr(rhs._rightLowAddr)
, _rightHighAddr(rhs._rightHighAddr)
, _min(rhs._min)
, _max(rhs._max)
, _default(rhs._default)
{
}

std::string Property::getName() const
{
	return _name;
}

int Property::getLeftLowAddr() const
{
	return _leftLowAddr;
}

int Property::getLeftHighAddr() const
{
	return _leftHighAddr;
}

int Property::getRightLowAddr() const
{
	return _rightLowAddr;
}

int Property::getRightHighAddr() const
{
	return _rightHighAddr;
}

int Property::getMin() const
{
	return _min;
}

int Property::getMax() const
{
	return _max;
}

int Property::getDefault() const
{
	return _default;
}
