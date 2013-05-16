#pragma once

/* STL */
#include <string>
#include <vector>

class Property
{
public:
	static std::vector<Property> getProperties();

	Property();
	Property(const std::string &name, int leftLowAddr, int leftHighAddr, int rightLowAddr, int rightHighAddr, int minValue, int maxValue, int defaultValue);
	Property(const Property &rhs);

	std::string getName() const;
	int getLeftLowAddr() const;
	int getLeftHighAddr() const;
	int getRightLowAddr() const;
	int getRightHighAddr() const;
	int getMin() const;
	int getMax() const;
	int getDefault() const;

protected:
	std::string _name;
	int _leftLowAddr;
	int _leftHighAddr;
	int _rightLowAddr;
	int _rightHighAddr;
	int _min;
	int _max;
	int _default;
};
