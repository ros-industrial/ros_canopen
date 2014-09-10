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
    virtual ~Layer();
};

class LayerVector{
    typedef std::vector<boost::shared_ptr<Layer> > vector_type ;
    vector_type layers;
    template<typename Iterator> bool call(bool(Layer::*func)(void), const Iterator &begin, const Iterator &end){
        bool res = true;
        for(Iterator it = begin; it != end; ++it){
            res = ((**it).*func)() && res;
        }
        return res;
    }
public:
    bool call_fwd(bool(Layer::*func)(void)){
        return call(func, layers.begin(), layers.end());
    }
    bool call_rev(bool(Layer::*func)(void)){
        return call(func, layers.rbegin(), layers.rend());
    }
    void add(const boost::shared_ptr<Layer> &l) { layers.push_back(l); }
};
    
class LayerStack : public Layer, public LayerVector{
public:
    virtual bool read() { return call_fwd(&Layer::read); }
    virtual bool write() { return call_rev(&Layer::write); }
    virtual bool report() { return call_fwd(&Layer::report); }
    virtual bool init() { return call_fwd(&Layer::init); }
    virtual bool recover() { return call_fwd(&Layer::recover); }
    virtual bool shutdown() { return call_rev(&Layer::shutdown); }
};
class LayerGroup : public Layer, public LayerVector{
public:
    virtual bool read() { return call_fwd(&Layer::read); }
    virtual bool write() { return call_fwd(&Layer::write); }
    virtual bool report() { return call_fwd(&Layer::report); }
    virtual bool init() { return call_fwd(&Layer::init); }
    virtual bool recover() { return call_fwd(&Layer::recover); }
    virtual bool shutdown() { return call_fwd(&Layer::shutdown); }
};

class CANLayer: public Layer{
    boost::shared_ptr<ipa_can::DriverInterface> driver_;
    const std::string device_;
    const unsigned int bitrate_;
public:
    CANLayer(const boost::shared_ptr<ipa_can::DriverInterface> &driver, const std::string &device, const unsigned int bitrate)
    : driver_(driver), device_(device), bitrate_(bitrate) { assert(driver_); }
    virtual bool read() { return driver_->getState().isReady(); }
    virtual bool write() { return driver_->getState().isReady(); }
    virtual bool report() { return false; }
    virtual bool init() { driver_->init(device_, bitrate_); }
    virtual bool recover() { return driver_->recover(); }
    virtual bool shutdown() { driver_->shutdown(); }
};
} // namespace ipa_canopen

#endif
