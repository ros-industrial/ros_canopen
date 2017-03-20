
#ifndef INTERFACE_MAPPING_H_
#define INTERFACE_MAPPING_H_

#include <string>

#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/foreach.hpp>

#include <canopen_402/base.h>

class InterfaceMapping {
    typedef boost::bimap<boost::bimaps::multiset_of<std::string>, boost::bimaps::set_of<canopen::MotorBase::OperationMode>  > bimap_type;
    bimap_type mapping_;
public:
    InterfaceMapping(){
        mapping_.insert(bimap_type::value_type("hardware_interface::PositionJointInterface" ,canopen::MotorBase::Profiled_Position));
        mapping_.insert(bimap_type::value_type("hardware_interface::PositionJointInterface" ,canopen::MotorBase::Interpolated_Position));
        mapping_.insert(bimap_type::value_type("hardware_interface::PositionJointInterface" ,canopen::MotorBase::Cyclic_Synchronous_Position));

        mapping_.insert(bimap_type::value_type("hardware_interface::VelocityJointInterface" ,canopen::MotorBase::Velocity));
        mapping_.insert(bimap_type::value_type("hardware_interface::VelocityJointInterface" ,canopen::MotorBase::Profiled_Velocity));
        mapping_.insert(bimap_type::value_type("hardware_interface::VelocityJointInterface" ,canopen::MotorBase::Cyclic_Synchronous_Velocity));

        mapping_.insert(bimap_type::value_type("hardware_interface::EffortJointInterface" ,canopen::MotorBase::Profiled_Torque));
        mapping_.insert(bimap_type::value_type("hardware_interface::EffortJointInterface" ,canopen::MotorBase::Cyclic_Synchronous_Torque));
    }
    std::vector<canopen::MotorBase::OperationMode> getInterfaceModes(const std::string &interface){
        std::vector<canopen::MotorBase::OperationMode> modes;
        BOOST_FOREACH(bimap_type::left_reference i, mapping_.left.equal_range(interface)){
            modes.push_back(i.second);
        }
        return modes;
    }
    bool hasConflict(const std::string &interface, canopen::MotorBase::OperationMode mode){
        bimap_type::right_const_iterator it;
        if((it = mapping_.right.find(mode)) != mapping_.right.end()){
            return it->second != interface;
        }
        return false;
    }
  
};

extern InterfaceMapping g_interface_mapping;

#endif /* INTERFACE_MAPPING_H_ */
