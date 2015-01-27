#ifndef FACTORY_PRODUCIBLE_HPP
#define FACTORY_PRODUCIBLE_HPP

#include <string>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <stdexcept>

namespace viennamini {

template < typename BaseClassT
         , typename TypeIdT
         , typename ConstructorDataT //not entirely sure about this one - what if we need more than one argument? how do initializer lists work (if at all)?
         , typename ExceptionT = std::runtime_error //this can be turned into a policy if needed
         , typename PtrT = std::shared_ptr<BaseClassT> //this can also be turned into a policy if needed
         >
class factory_producible
{
public:
  virtual ~factory_producible() = 0;

  static PtrT create(TypeIdT const& type_id, ConstructorDataT const& cd);

  template <typename ConcreteDerivedT>
  static bool register_at_factory(TypeIdT const& type_id);

private:
  class factory
  {
  public:
    PtrT create_product(TypeIdT const& type_id, ConstructorDataT const& cd) const;

    template <typename ProductT>
    bool register_product(TypeIdT const& type_id);

  private:
    typedef std::function<BaseClassT* (ConstructorDataT const&)> FactoryFunction;
    typedef std::unordered_map<TypeIdT, FactoryFunction> FactoryFunctionAssociativeContainer;

    FactoryFunctionAssociativeContainer factory_functions_;
  };

  static factory& instance(); //uses meyer's singleton, threadsafety is a potential concern
};

// Implementation

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>::~factory_producible()
{
}

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
PtrT factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>
  ::create(TypeIdT const& type_id, ConstructorDataT const& cd)
{
  return instance().create_product(type_id, cd);
}

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
template <typename ConcreteDerivedT>
bool factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>
  ::register_at_factory(TypeIdT const& type_id)
{
  return instance().template register_product<ConcreteDerivedT>(type_id);
}

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
PtrT factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>
  ::factory::create_product(TypeIdT const& type_id, ConstructorDataT const& cd) const
{
  auto it = factory_functions_.find(type_id);
  if(it == std::end(factory_functions_))
  {
    std::stringstream sstr;
    sstr << "Unknown TypeId: " << type_id << " ";
    sstr << "known TypeIds:";
    for(const auto& tit : factory_functions_)
    {
      sstr << " " << tit.first;
    }
    throw ExceptionT(sstr.str());
  }

  return PtrT((it->second)(cd));
}

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
template <class ProductT>
bool factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>::factory::register_product(TypeIdT const& type_id)
{
  auto it = factory_functions_.find(type_id);
  if(it == std::end(factory_functions_))
  {
    factory_functions_[type_id] = FactoryFunction([](ConstructorDataT const& cd) {return new ProductT(cd);});
    return true;
  }

  return false;
}

template <typename BaseClassT, typename TypeIdT, typename ConstructorDataT, typename ExceptionT, typename PtrT>
typename factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>::factory&
factory_producible<BaseClassT, TypeIdT, ConstructorDataT, ExceptionT, PtrT>::instance()
{
  static factory factory_singleton_instance;
  return factory_singleton_instance;
}

} //end of namespace viennamini

#endif
