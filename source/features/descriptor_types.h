#ifndef DESCRIPTOR_TYPES
#define DESCRIPTOR_TYPES

#include <vector>

template <Scalar>
struct CustomSizeDescriptor 
{
public:
	CustomSizeDescriptor(uint size)
		: size_(size), vec_descriptor(size), descriptor(vec_descriptor.data()) {}
	
	inline uint getSize() { return size_ } const;

// order of fields matters because descriptor is const pointer
private:
	uint size_;
	std::vector<Scalar> vec_descriptor;

public:
	Scalar* const descriptor;

};

typedef CustomSizeDescriptor<uchar> CustomSizeBinaryDescriptor;
typedef CustomSizeDescriptor<float> CustomSizeRealDescriptor;

#endif // DESCRIPTOR_TYPES