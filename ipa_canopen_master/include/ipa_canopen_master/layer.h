#ifndef H_IPA_CANOPEN_LAYER
#define H_IPA_CANOPEN_LAYER

#include <vector>
#include <boost/shared_ptr.hpp>
namespace ipa_canopen{

class Layer{
public:
    virtual bool read() = 0;
    virtual bool write() = 0;
    virtual bool report() = 0;
    virtual bool init() = 0;
    virtual bool recover() = 0;
    virtual bool shutdown() = 0;
    virtual ~Layer() {}
};

template<typename T> class LayerVector{
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    template<typename Iterator> bool call(bool(T::*func)(void), const Iterator &begin, const Iterator &end){
        bool res = true;
        for(Iterator it = begin; it != end; ++it){
            res = ((**it).*func)() && res;
        }
        return res;
    }
public:
    bool call_fwd(bool(T::*func)(void)){
        return call(func, layers.begin(), layers.end());
    }
    bool call_rev(bool(T::*func)(void)){
        return call(func, layers.rbegin(), layers.rend());
    }
    void add(const boost::shared_ptr<T> &l) { layers.push_back(l); }
};
    
class LayerStack : public Layer, public LayerVector<Layer>{
public:
    virtual bool read() { return this->call_fwd(&Layer::read); }
    virtual bool write() { return this->call_rev(&Layer::write); }
    virtual bool report() { return this->call_fwd(&Layer::report); }
    virtual bool init() { return this->call_fwd(&Layer::init); }
    virtual bool recover() { return this->call_fwd(&Layer::recover); }
    virtual bool shutdown() { return this->call_rev(&Layer::shutdown); }
};
template<typename T> class LayerGroup : public Layer, public LayerVector<T>{
public:
    virtual bool read() { return this->call_fwd(&Layer::read); }
    virtual bool write() { return this->call_fwd(&Layer::write); }
    virtual bool report() { return this->call_fwd(&Layer::report); }
    virtual bool init() { return this->call_fwd(&Layer::init); }
    virtual bool recover() { return this->call_fwd(&Layer::recover); }
    virtual bool shutdown() { return this->call_fwd(&Layer::shutdown); }
};

template<typename Driver> class CANLayer: public Layer{
    boost::shared_ptr<Driver> driver_;
    const std::string device_;
    const unsigned int bitrate_;
public:
    CANLayer(const boost::shared_ptr<Driver> &driver, const std::string &device, const unsigned int bitrate)
    : driver_(driver), device_(device), bitrate_(bitrate) { assert(driver_); }
    virtual bool read() { return driver_->getState().isReady(); }
    virtual bool write() { return driver_->getState().isReady(); }
    virtual bool report() { return false; }
    virtual bool init() { return driver_->init(device_, bitrate_); }
    virtual bool recover() { return driver_->recover(); }
    virtual bool shutdown() { driver_->shutdown(); return true;}
};
} // namespace ipa_canopen

#endif
