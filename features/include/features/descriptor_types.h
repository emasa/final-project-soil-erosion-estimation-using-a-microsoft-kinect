#ifndef DESCRIPTOR_TYPES
#define DESCRIPTOR_TYPES

#include <vector>

#include <pcl/point_types.h>

struct ORBSignature32
{
	unsigned char descriptor[32];

	// friend std::ostream& operator << (std::ostream& os, const ORBSignature32& p);
};

struct SURFSignature64
{
	float descriptor[64];
	
	// friend std::ostream& operator << (std::ostream& os, const SURFSignature64& p);
};

struct SURFSignature128
{
	float descriptor[128];
	
	// friend std::ostream& operator << (std::ostream& os, const SURFSignature128& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT ( 
   ORBSignature32,  
   (unsigned char[64], descriptor, descriptor) 
);

POINT_CLOUD_REGISTER_POINT_STRUCT ( 
   SURFSignature64,  
   (float[64], descriptor, descriptor) 
);

POINT_CLOUD_REGISTER_POINT_STRUCT ( 
   SURFSignature128,  
   (float[128], descriptor, descriptor) 
);

#endif // DESCRIPTOR_TYPES