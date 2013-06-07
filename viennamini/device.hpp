#ifndef VIENNAMINI_DEVICE_HPP
#define VIENNAMINI_DEVICE_HPP

namespace viennamini {


//template <typename ConfigType>
//void assign(viennamini::contact_key           const& contact,
//            viennagrid::segment_t<ConfigType> const& seg,
//            std::string                       const& material_string)
//{
//    viennamini::permittivity_key       eps;
//    viennamini::builtin_potential_key  builtin;
//    viennamini::material_key           material;

//    viennafvm::set_quantity_region(eps,     domain.segments()[si], false);
//    viennafvm::set_quantity_region(builtin, domain.segments()[si], false);
    
//    viennadata::access<viennamini::material_key, std::string >(material)(seg) = material_string;
//}

//template <typename ConfigType>
//void assign(viennamini::oxide_key             const& oxide,
//            viennagrid::segment_t<ConfigType> const& seg,
//            std::string                       const& material_string)
//{
//    viennamini::permittivity_key       eps;
//    viennamini::builtin_potential_key  builtin;
//    viennamini::material_key           material;

//    viennafvm::set_quantity_region(eps, seg, true);
//    viennafvm::set_quantity_value (eps, seg,
//                                   matlib.getParameterValue(
//                                       siter->material.toStdString(), "permittivity") * eps0);

//    viennafvm::set_quantity_region(builtin_key, domain.segments()[si], false); // TODO not for oxides?!

//    viennafvm::disable_quantity(domain.segments()[si], my_simulator.quantity_electron_density());
//    viennafvm::disable_quantity(domain.segments()[si], my_simulator.quantity_hole_density());
//}

} // viennamini

#endif

