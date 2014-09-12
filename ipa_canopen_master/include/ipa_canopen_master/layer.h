#ifndef H_IPA_CANOPEN_LAYER
#define H_IPA_CANOPEN_LAYER

#include <vector>
#include <boost/shared_ptr.hpp>
namespace ipa_canopen{

class LayerStatus{
public:
    enum State{
        OK = 0, WARN = 1, ERROR= 2, STALE = 3, UNDEFINED = -1, UNBOUNDED = 3
    };
    const State& get() const{
        return state;
    }
    bool bounded(const State &s) const{
        return state != UNDEFINED && state <= s;
    }
    void set(const State &s) {
        if(s > state) state = s;
    }
    LayerStatus() : state(UNDEFINED) {}
    void reset() { state = UNDEFINED; }
private:
    State state;
};

class LayerStatusExtended : public LayerStatus{
    std::vector<std::string> reasons_;
    std::vector<std::pair<std::string, std::string> > values_;
public:
    const std::vector<std::string> &reasons() { return reasons_; }
    const std::vector<std::pair<std::string, std::string> > &values() { return values_; }
    
    void reason(const std::string &reason) {
        reasons_.push_back(reason);
    }
    void add(const std::string &key, const std::string &value){
        values_.push_back(std::make_pair(key,value));
    }
    template<typename T> void add_typed(const std::string &key, const T &value){
        std::stringstream str;
        str << value;
        add(key, str.str());
    }
};

class Layer{
public:
    virtual void read(LayerStatus &status) = 0;
    virtual void write(LayerStatus &status) = 0;
    
    virtual void report(LayerStatusExtended &status) = 0;
    
    virtual void init(LayerStatusExtended &status) = 0;
    virtual void shutdown(LayerStatus &status) = 0;
    
    virtual void halt(LayerStatus &status) {} // TODO
    virtual void recover(LayerStatusExtended &status) = 0;
    
    virtual ~Layer() {}
};

class SimpleLayer: public Layer {
    void adapt(bool (SimpleLayer::*func) (void), LayerStatus &status){
        if (!(this->*func)()) status.set(LayerStatus::ERROR);
    }
public:
    virtual void read(LayerStatus &status) { adapt(&SimpleLayer::read, status); }
    virtual void write(LayerStatus &status) { adapt(&SimpleLayer::write, status); }
    virtual void report(LayerStatusExtended &status) { adapt(&SimpleLayer::report, status); }
    virtual void init(LayerStatusExtended &status) { adapt(&SimpleLayer::init, status); }
    virtual void recover(LayerStatusExtended &status) { adapt(&SimpleLayer::recover, status); }
    virtual void shutdown(LayerStatus &status) {  adapt(&SimpleLayer::shutdown, status); }
    
    virtual bool read()  = 0;
    virtual bool write()  = 0;
    virtual bool report()  = 0;
    virtual bool init()  = 0;
    virtual bool recover()  = 0;
    virtual bool shutdown()  = 0;
};

template<typename T> class VectorHelper{
protected:
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    
    template<typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end, const LayerStatus::State &bound = LayerStatus::UNBOUNDED){
        for(Iterator it = begin; it != end; ++it){
            ((**it).*func)(status);
            if(!status.bounded(bound)){
                return it;
            }
        }
        return end;

    }
public:
    void add(const boost::shared_ptr<T> &l) { layers.push_back(l); }
};
    
class LayerStack : public Layer, public VectorHelper<Layer>{
public:
    virtual void read(LayerStatus &status){
        vector_type::iterator it = call(&Layer::read, status, layers.begin(), layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != layers.end()) call(&Layer::halt, omit, layers.rbegin(), vector_type::reverse_iterator(it) +1);
    }
    virtual void write(LayerStatus &status){
        vector_type::reverse_iterator it = call(&Layer::write, status, layers.rbegin(), layers.rend(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != layers.rend()) call(&Layer::halt, omit, layers.rbegin(), vector_type::reverse_iterator(it) +1);
    }
    virtual void report(LayerStatusExtended &status){
        vector_type::iterator it = call(&Layer::report, status, layers.begin(), layers.end());
    }
    virtual void init(LayerStatusExtended &status) {
        vector_type::iterator it = call(&Layer::init, status, layers.begin(), layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != layers.end()) call(&Layer::shutdown, omit, vector_type::reverse_iterator(it), layers.rend());
    }
    virtual void recover(LayerStatusExtended &status){
        vector_type::iterator it = call(&Layer::recover, status, layers.begin(), layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != layers.end()) call(&Layer::halt, omit, vector_type::reverse_iterator(it), layers.rend());
    }
    virtual void shutdown(LayerStatus &status){
        call(&Layer::shutdown, status, layers.rbegin(), layers.rend());
    }
    virtual void halt(LayerStatus &status){
        call(&Layer::halt, status, layers.rbegin(), layers.rend());
    }
};

template<typename T> class LayerGroup : public Layer, public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void read(LayerStatus &status){
        typename V::vector_type::iterator it = this->call(&Layer::read, status, this->layers.begin(), this->layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != this->layers.end()) this->call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
    }
    virtual void write(LayerStatus &status){
        typename V::vector_type::iterator it = this->call(&Layer::write, status, this->layers.begin(), this->layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != this->layers.end()) this->call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
    }
    virtual void report(LayerStatusExtended &status){
        this->call(&Layer::report, status, this->layers.begin(), this->layers.end());
    }
    virtual void init(LayerStatusExtended &status) {
        typename V::vector_type::iterator it = this->call(&Layer::recover, status, this->layers.begin(), this->layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != this->layers.end()) this->call(&Layer::shutdown, omit, this->layers.begin(), this->layers.end());
    }
    virtual void recover(LayerStatusExtended &status){
        typename V::vector_type::iterator it = this->call(&Layer::recover, status, this->layers.begin(), this->layers.end(), LayerStatus::WARN);
        LayerStatus omit;
        if(it != this->layers.end()) this->call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
    }
    virtual void shutdown(LayerStatus &status){
        this->call(&Layer::shutdown, status, this->layers.begin(), this->layers.end());
    }
    virtual void halt(LayerStatus &status){
        this->call(&Layer::halt, status, this->layers.begin(), this->layers.end());
    }
};

template<typename Driver> class CANLayer: public SimpleLayer{ // TODO: implement Layer
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
