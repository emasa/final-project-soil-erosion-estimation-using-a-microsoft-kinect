#ifndef DESCRIPTOR_REPRESENTATIONS
#define DESCRIPTOR_REPRESENTATIONS

#include <memory>

#include <pcl/point_representation.h>

#include "descriptor_types.h"

template <typename PointT>
class BinaryPointRepresentation
{
	protected:
		int nr_dimensions_;
		bool trivial_;

	public:
		typedef std::shared_pointer<BinaryPointRepresentation<PointT>> Ptr;
		typedef std::shared_pointer<const BinaryPointRepresentation<PointT>> ConstPtr;

		BinaryPointRepresentation() : nr_dimensions_(0), trivial_(false) {}
		
		virtual ~BinaryPointRepresentation {}

		virtual void copyToBinaryArray(const PointT& p, uchar* out) const = 0;

		inline bool isTrivial() const { return trivial_; }

        virtual bool isValid (const PointT &p) const = 0;

		template <typename OutputType> void
		vectorize (const PointT &p, OutputType &out) const
		{
		  	uchar *temp = new uchar[nr_dimensions_];
		  	copyToBinaryArray (p, temp);
		    
		    for (int i = 0; i < nr_dimensions_; ++i)
		   		out[i] = temp[i];
			
			delete[] temp;
		}

		inline int getNumberOfDimensions () const { return (nr_dimensions_); }
};

class CustomSizeBinaryDescriptorRepresentation : BinaryPointRepresentation<CustomSizeBinaryDescriptor>
{
	
	public:
		typedef std::shared_pointer<CustomSizeBinaryDescriptorRepresentation> Ptr;
		typedef std::shared_pointer<const CustomSizeBinaryDescriptorRepresentation> ConstPtr;

	CustomSizeBinaryDescriptorRepresentation(int nr_dimensions) 
	{
		nr_dimensions_ = nr_dimensions;
	}

	bool 
	isValid (const CustomSizeBinaryDescriptor &p) const { return true };

	void 
	copyToBinaryArray(const CustomSizeBinaryDescriptor& p, uchar* out) const
	{
		for (int i = 0; i < nr_dimensions_; ++i)
			out[i] = p.descriptor[i];
	}

	inline Ptr
	makeShared () const
	{
		return (Ptr (new CustomSizeBinaryDescriptorRepresentation<PointT> (*this)));
	}
};

// TODO: definir representacion para pcl::BRISKSignature512

///////////////////////////////////////////////////////////////////////////////

class CustomSizeDescriptorRepresentation : public PointRepresentation<CustomSizeRealDescriptor>
{	
	public:
		typedef std::shared_pointer<CustomSizeDescriptorRepresentation> Ptr;
		typedef std::shared_pointer<const CustomSizeDescriptorRepresentation> ConstPtr;
	
	CustomSizeDescriptorRepresentation (int nr_dimensions)
	{
		nr_dimensions_ = nr_dimensions;
	}

	void 
	copyToFloatArray (const CustomSizeRealDescriptor &p, float * out) const
	{
		for (int i = 0; i < nr_dimensions_; ++i)
			out[i] = p.descriptor[i];
	}

	inline Ptr
	makeShared () const
	{
		return (Ptr (new CustomSizeDescriptorRepresentation(*this)));
	}
};

#endif // DESCRIPTOR_REPRESENTATIONS