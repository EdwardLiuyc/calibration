#ifndef REGISTRATORS_KFPCS_H_
#define REGISTRATORS_KFPCS_H_

namespace l2l_calib{
namespace registrator{

template <typename PointType>
class KfpcsUsingPcl
 : public RegistratorInterface<PointType> {

public:
  KfpcsUsingPcl() 
   : RegistratorInterface<PointType>()
  {
    RegistratorInterface<PointType>::type_ = kKfpcsPcl;
  }

  ~KfpcsUsingPcl() = default;

private:

};

}
}


#endif // REGISTRATORS_KFPCS_H_