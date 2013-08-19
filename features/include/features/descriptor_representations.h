#ifndef DESCRIPTOR_REPRESENTATIONS
#define DESCRIPTOR_REPRESENTATIONS

#include <pcl/point_representation.h>
#include <boost/shared_ptr.hpp>

#include "descriptor_types.h"

template <typename PointT>
class BinaryPointRepresentation
{
	public:
		typedef boost::shared_ptr<BinaryPointRepresentation<PointT>> Ptr;
		typedef boost::shared_ptr<const BinaryPointRepresentation<PointT>> ConstPtr;

		BinaryPointRepresentation() : nr_dimensions_(0), trivial_(false) {}
		virtual ~BinaryPointRepresentation() {}
		
		virtual void copyToBinaryArray(const PointT& p, unsigned char* out) const = 0;

		inline bool 
		isTrivial() const { return trivial_; }

        virtual bool isValid (const PointT &p) const = 0;

		template <typename OutputType> void
		vectorize (const PointT &p, OutputType &out) const
		{
		  	unsigned char *temp = new unsigned char[nr_dimensions_];
		  	copyToBinaryArray (p, temp);
		    
		    for (int i = 0; i < nr_dimensions_; ++i)
		   		out[i] = temp[i];
			
			delete[] temp;
		}

		inline int 
		getNumberOfDimensions () const { return nr_dimensions_; }

	protected:
		int nr_dimensions_;
		bool trivial_;
};

class CustomSizeBinaryDescriptorRepresentation : BinaryPointRepresentation<CustomSizeBinaryDescriptor>
{
	public:
		typedef boost::shared_ptr<CustomSizeBinaryDescriptorRepresentation> Ptr;
		typedef boost::shared_ptr<const CustomSizeBinaryDescriptorRepresentation> ConstPtr;

	CustomSizeBinaryDescriptorRepresentation(int nr_dimensions) 
	{
		nr_dimensions_ = nr_dimensions;
		trivial_ = false;
	}

	bool 
	isValid (const CustomSizeBinaryDescriptor &p) const { return true; }

	void 
	copyToBinaryArray(const CustomSizeBinaryDescriptor& p, unsigned char* out) const
	{
		for (int i = 0; i < nr_dimensions_; ++i)
			out[i] = p.descriptor.at(i); // bounds checking
	}

	inline Ptr
	makeShared () const
	{
		return (Ptr (new CustomSizeBinaryDescriptorRepresentation(*this)));
	}
};

// TODO: definir representacion para pcl::BRISKSignature512

///////////////////////////////////////////////////////////////////////////////

class CustomSizeDescriptorRepresentation : public pcl::PointRepresentation<CustomSizeRealDescriptor>
{	
	public:
		typedef boost::shared_ptr<CustomSizeDescriptorRepresentation> Ptr;
		typedef boost::shared_ptr<const CustomSizeDescriptorRepresentation> ConstPtr;
	
	CustomSizeDescriptorRepresentation (int nr_dimensions)
	{
		nr_dimensions_ = nr_dimensions;
	}

	void 
	copyToFloatArray (const CustomSizeRealDescriptor &p, float * out) const
	{
		for (int i = 0; i < nr_dimensions_; ++i)
			out[i] = p.descriptor.at(i); // bounds checking
	}

	inline Ptr
	makeShared () const
	{
		return (Ptr (new CustomSizeDescriptorRepresentation(*this)));
	}
};

#endif // DESCRIPTOR_REPRESENTATIONS