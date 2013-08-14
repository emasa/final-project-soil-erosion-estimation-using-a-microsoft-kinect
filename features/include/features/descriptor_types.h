#ifndef DESCRIPTOR_TYPES
#define DESCRIPTOR_TYPES

#include <vector>

struct CustomSizeBinaryDescriptor 
{
	CustomSizeBinaryDescriptor(uint size=0): descriptor(size) {}	
	std::vector<unsigned char> descriptor;
};

struct CustomSizeRealDescriptor 
{
	CustomSizeRealDescriptor(uint size=0): descriptor(size) {}	
	std::vector<float> descriptor;
};

#endif // DESCRIPTOR_TYPES