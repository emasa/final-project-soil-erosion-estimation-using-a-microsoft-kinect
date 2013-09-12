#ifndef DESCRIPTOR_REPRESENTATIONS
#define DESCRIPTOR_REPRESENTATIONS
#define PCL_NO_PRECOMPILE

#include <pcl/point_representation.h>
#include <boost/shared_ptr.hpp>

#include "features/descriptor_types.h"

namespace pcl
{

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

		bool 
		isValid (const PointT &p) const { return true; }

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

class ORBSignature32Representation : public BinaryPointRepresentation<ORBSignature32>
{
public:
	typedef boost::shared_ptr<ORBSignature32Representation> Ptr;
	typedef boost::shared_ptr<const ORBSignature32Representation> ConstPtr;

	ORBSignature32Representation()
	{
		nr_dimensions_ = 32;
		trivial_ = false;
	}

	void 
	copyToBinaryArray(const ORBSignature32& p, unsigned char* out) const
	{
		for (int i = 0; i < nr_dimensions_; ++i)
			out[i] = p.descriptor[i];
	}

	inline Ptr
	makeShared () const
	{
		return (Ptr (new ORBSignature32Representation(*this)));
	}
};

// TODO: definir representacion para pcl::BRISKSignature512

///////////////////////////////////////////////////////////////////////////////

template <>
class DefaultPointRepresentation<SURFSignature64> : public DefaultFeatureRepresentation<SURFSignature64>
{};

template <>
class DefaultPointRepresentation<SURFSignature128> : public DefaultFeatureRepresentation<SURFSignature128>
{
	// DefaultPointRepresentation
	// {
	// 	nr_dimensions_ = 128;
	// 	trivial_ = false;
	// }

	// void 
	// copyToFloatArray(const SURFSignature128& p, float* out) const
	// {
	// 	for (int i = 0; i < nr_dimensions_; ++i)
	// 		out[i] = p.descriptor[i];
	// }
};

}

#endif // DESCRIPTOR_REPRESENTATIONS