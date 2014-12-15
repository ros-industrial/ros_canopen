#ifndef H_CANOPEN_LAYER
#define H_CANOPEN_LAYER

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>

namespace canopen{

class LayerStatus{
    mutable boost::mutex write_mutex_;
    enum State{
        OK = 0, WARN = 1, ERROR= 2, STALE = 3, UNBOUNDED = 3
    };
    boost::atomic<State> state;
    void set(const State &s){
        boost::mutex::scoped_lock lock(write_mutex_);
        if(s > state) state = s;
    }
    std::string reason_;
    void reason(const std::string &r){
        if(!r.empty()){
            boost::mutex::scoped_lock lock(write_mutex_);
            if(reason_.empty())  reason_ = r;
            else reason_ += "; " + r;
        }
    }
public:
    struct Ok { static const State state = OK; private: Ok();};
    struct Warn { static const State state = WARN; private: Warn(); };
    struct Error { static const State state = ERROR; private: Error(); };
    struct Stale { static const State state = STALE; private: Stale(); };
    struct Unbounded { static const State state = UNBOUNDED; private: Unbounded(); };

    template<typename T> bool bounded() const{ return state <= T::state; }
    
    LayerStatus() : state(OK) {}
    
    int get() const { return state; }
    
    const std::string reason() const { boost::mutex::scoped_lock lock(write_mutex_); return reason_; }

    const void warn(const std::string & r = "") { reason(r); set(WARN); }
    const void error(const std::string & r = "") { reason(r); set(ERROR); }
    const void stale(const std::string & r = "") { reason(r); set(STALE); }
};
class LayerReport : public LayerStatus {
    std::vector<std::pair<std::string, std::string> > values_;
public:
    const std::vector<std::pair<std::string, std::string> > &values() const { return values_; }
    template<typename T> void add(const std::string &key, const T &value) {
        std::stringstream str;
        str << value;
        values_.push_back(std::make_pair(key,str.str()));
    }
};
class Layer{
public:
    const std::string name;
    virtual void pending(LayerStatus &status) {}
    virtual void read(LayerStatus &status) = 0;
    virtual void write(LayerStatus &status) = 0;
    
    virtual void diag(LayerReport &report) = 0;
    
    virtual void init(LayerStatus &status) = 0;
    virtual void shutdown(LayerStatus &status) = 0;
    
    virtual void halt(LayerStatus &status) {} // TODO
    virtual void recover(LayerStatus &status) = 0;
    
    Layer(const std::string &n) : name(n) {}
    
    virtual ~Layer() {}
};

class SimpleLayer: public Layer {
    void adapt(bool (SimpleLayer::*func) (void), LayerStatus &status){
        if(!(this->*func)()) status.error();
    }
public:
        
    virtual void read(LayerStatus &status) { adapt(&SimpleLayer::read, status); }
    virtual void write(LayerStatus &status) { adapt(&SimpleLayer::write, status); }
    virtual void diag(LayerReport &report) { adapt(&SimpleLayer::report, report); }
    virtual void init(LayerStatus &status) { adapt(&SimpleLayer::init, status); }
    virtual void recover(LayerStatus &status) { adapt(&SimpleLayer::recover, status); }
    virtual void shutdown(LayerStatus &status) {  adapt(&SimpleLayer::shutdown, status); }
    
    virtual bool read()  = 0;
    virtual bool write()  = 0;
    virtual bool report()  = 0;
    virtual bool init()  = 0;
    virtual bool recover()  = 0;
    virtual bool shutdown()  = 0;
    
    SimpleLayer(const std::string &n) : Layer(n) {}
    SimpleLayer() : Layer("NO NAME GIVEN") {}
};

template<typename T> class VectorHelper{
protected:
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    
    template<typename Bound, typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        for(Iterator it = begin; it != end; ++it){
            ((**it).*func)(status);
            if(!status.template bounded<Bound>()){
                return it;
            }
        }
        return end;
    }
    template<typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        return call<LayerStatus::Unbounded, Iterator, Data>(func, status, begin, end);
    }
public:
    void add(const boost::shared_ptr<T> &l) { layers.push_back(l); }
};
    
class LayerStack : public Layer, public VectorHelper<Layer>{
    boost::mutex end_mutex_;
    vector_type::iterator run_end_;
    
    void bringup(void(Layer::*func)(LayerStatus&), void(Layer::*func_fail)(LayerStatus&), LayerStatus &status){
        vector_type::iterator it = layers.begin();
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = it;
        }
        for(; it != layers.end(); ++it){
            {
                boost::mutex::scoped_lock lock(end_mutex_);
                run_end_ = it;
            }
            ((**it).*func)(status);
            if(!status.bounded<LayerStatus::Warn>()) break;
        }
        if(it != layers.end()){
            LayerStatus omit;
            call(func_fail, omit, vector_type::reverse_iterator(it), layers.rend());
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = layers.begin();
        }
        else{
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = layers.end();
        }
    }
public:
    virtual void read(LayerStatus &status){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            if(end == run_end_){
                run_end_ = layers.begin(); // reset
            }
            end = run_end_;
        }
        vector_type::iterator it = call<LayerStatus::Warn>(&Layer::read, status, layers.begin(), end);
        if(it != end){
            LayerStatus omit;
            call(&Layer::halt, omit, layers.rbegin(), vector_type::reverse_iterator(it));
        }
    }
    virtual void pending(LayerStatus &status){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            end = run_end_;
        }
        if(end != layers.end()){
            (**end).pending(status);
        }
    }
    virtual void write(LayerStatus &status){
        vector_type::reverse_iterator begin;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            begin = vector_type::reverse_iterator(run_end_);
        }
        vector_type::reverse_iterator it = call(&Layer::write, status, begin, layers.rend());
        if(it != layers.rend()){
            LayerStatus omit;
            call(&Layer::halt, omit, begin, vector_type::reverse_iterator(it));
        }
    }
    virtual void diag(LayerReport &report){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            if(end == run_end_) return;
            end = run_end_;
        }
        vector_type::iterator it = call(&Layer::diag, report, layers.begin(), end);
    }
    virtual void init(LayerStatus &status) {
        bringup(&Layer::init, &Layer::shutdown, status);
    }
    virtual void recover(LayerStatus &status) {
        bringup(&Layer::recover, &Layer::halt, status);
    }
    virtual void shutdown(LayerStatus &status){
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = layers.begin();
        }
        call(&Layer::shutdown, status, layers.rbegin(), layers.rend());
    }
    virtual void halt(LayerStatus &status){
        call(&Layer::halt, status, layers.rbegin(), layers.rend());
    }

    LayerStack(const std::string &n) : Layer(n) {}
};

template<typename T> class LayerGroup : public Layer, public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void pending(LayerStatus &status){
        this->template call<LayerStatus::Warn>(&Layer::pending, status, this->layers.begin(), this->layers.end());
    }
    virtual void read(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::read, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void write(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::write, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report, this->layers.begin(), this->layers.end());
    }
    virtual void init(LayerStatus &status) {
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::init, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::shutdown, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void recover(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::recover, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void shutdown(LayerStatus &status){
        this->template call(&Layer::shutdown, status, this->layers.begin(), this->layers.end());
    }
    virtual void halt(LayerStatus &status){
        this->template call(&Layer::halt, status, this->layers.begin(), this->layers.end());
    }
    LayerGroup(const std::string &n) : Layer(n) {}
};

} // namespace canopen

#endif
