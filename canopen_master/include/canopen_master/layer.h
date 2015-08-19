#ifndef H_CANOPEN_LAYER
#define H_CANOPEN_LAYER

#include <vector>
#include <boost/thread/shared_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace canopen{

class LayerStatus{
    mutable boost::mutex write_mutex_;
    enum State{
        OK = 0, WARN = 1, ERROR= 2, STALE = 3, UNBOUNDED = 3
    };
    boost::atomic<State> state;
    std::string reason_;

    virtual void set(const State &s, const std::string &r){
        boost::mutex::scoped_lock lock(write_mutex_);
        if(s > state) state = s;
        if(!r.empty()){
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

    const void warn(const std::string & r) { set(WARN, r); }
    const void error(const std::string & r) { set(ERROR, r); }
    const void stale(const std::string & r) { set(STALE, r); }
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

#define CATCH_LAYER_HANDLER_EXCEPTIONS(command, status)                           \
    try{ command; }                                                             \
    catch(std::exception &e) {status.error(boost::diagnostic_information(e)); }
  
class Layer{
public:
    enum LayerState{
        Off,
        Init,
        Shutdown,
        Error,
        Halt,
        Recover,
        Ready
    };

    const std::string name;

    void read(LayerStatus &status) {
        if(state > Off) CATCH_LAYER_HANDLER_EXCEPTIONS(handleRead(status, state), status);
    }
    void write(LayerStatus &status) {
        if(state > Off) CATCH_LAYER_HANDLER_EXCEPTIONS(handleWrite(status, state), status);
    }
    void diag(LayerReport &report) {
        if(state > Shutdown) CATCH_LAYER_HANDLER_EXCEPTIONS(handleDiag(report), report);
    }
    void init(LayerStatus &status) {
        if(state == Off){
            if(status.bounded<LayerStatus::Warn>()){
                state = Init;
                CATCH_LAYER_HANDLER_EXCEPTIONS(handleInit(status), status);
            }
            if(!status.bounded<LayerStatus::Warn>()) shutdown(status);
            else state = Ready;
        }
    }
    void shutdown(LayerStatus &status) {
        if(state != Off){
            state = Shutdown;
            CATCH_LAYER_HANDLER_EXCEPTIONS(handleShutdown(status), status);
            state = Off;
        }
    }
    void halt(LayerStatus &status) {
        if(state > Halt){
            state = Halt;
            CATCH_LAYER_HANDLER_EXCEPTIONS(handleHalt(status), status);
            state = Error;
        }
    }
    void recover(LayerStatus &status) {
        if(state == Error){
            if(status.bounded<LayerStatus::Warn>()){
                state = Recover;
                CATCH_LAYER_HANDLER_EXCEPTIONS(handleRecover(status), status);
            }
            if(!status.bounded<LayerStatus::Warn>()){
                halt(status);
            }else{
                state = Ready;
            }
        }

    }

    LayerState getLayerState() { return state; }

    Layer(const std::string &n) : name(n), state(Off) {}
    
    virtual ~Layer() {}

protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state)  = 0;
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state)  = 0;

    virtual void handleDiag(LayerReport &report)  = 0;

    virtual void handleInit(LayerStatus &status)  = 0;
    virtual void handleShutdown(LayerStatus &status)  = 0;

    virtual void handleHalt(LayerStatus &status)  = 0;
    virtual void handleRecover(LayerStatus &status)  = 0;

private:
    boost::atomic<LayerState> state;

};

template<typename T> class VectorHelper{
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    boost::shared_mutex mutex;

    template<typename Bound, typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        bool okay_on_start = status.template bounded<Bound>();

        for(Iterator it = begin; it != end; ++it){
            ((**it).*func)(status);
            if(okay_on_start && !status.template bounded<Bound>()){
                return it;
            }
        }
        return end;
    }
    template<typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        return call<LayerStatus::Unbounded, Iterator, Data>(func, status, begin, end);
    }
protected:
    template<typename Bound, typename Data> typename vector_type::iterator call(void(Layer::*func)(Data&), Data &status){
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return call<Bound>(func, status, layers.begin(), layers.end());
    }
    template<typename Data> typename vector_type::iterator call(void(Layer::*func)(Data&), Data &status){
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return call<LayerStatus::Unbounded>(func, status, layers.begin(), layers.end());
    }
    template<typename Bound, typename Data> typename vector_type::reverse_iterator call_rev(void(Layer::*func)(Data&), Data &status){
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return call<Bound>(func, status, layers.rbegin(), layers.rend());
    }
    template<typename Data> typename vector_type::reverse_iterator call_rev(void(Layer::*func)(Data&), Data &status){
        boost::shared_lock<boost::shared_mutex> lock(mutex);
        return call<LayerStatus::Unbounded>(func, status, layers.rbegin(), layers.rend());
    }
    void destroy() { boost::unique_lock<boost::shared_mutex> lock(mutex); layers.clear(); }
public:
    virtual void add(const boost::shared_ptr<T> &l) { boost::unique_lock<boost::shared_mutex> lock(mutex); layers.push_back(l); }
};

template<typename T=Layer> class LayerGroup : public Layer, public VectorHelper<T> {
protected:
    template<typename Data> void call_or_fail(void(Layer::*func)(Data&), void(Layer::*fail)(Data&), Data &status){
        bool wasError = !status.template bounded<LayerStatus::Warn>();
        this->template call(func, status);
        if(!wasError && !status.template bounded<LayerStatus::Warn>()){
            this->template call(fail, status);
            (this->*fail)(status);
        }
    }
    template<typename Data> void call_or_fail_rev(void(Layer::*func)(Data&), void(Layer::*fail)(Data&), Data &status){
        bool wasError = !status.template bounded<LayerStatus::Warn>();
        this->template call_rev(func, status);
        if(!wasError && !status.template bounded<LayerStatus::Warn>()){
            this->template call_rev(fail, status);
            (this->*fail)(status);
        }
    }

    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        this->template call_or_fail(&Layer::read, &Layer::halt, status);
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        this->template call_or_fail(&Layer::write, &Layer::halt, status);
    }

    virtual void handleDiag(LayerReport &report) { this->template call(&Layer::diag, report); }

    virtual void handleInit(LayerStatus &status) { this->template call<LayerStatus::Warn>(&Layer::init, status); }
    virtual void handleShutdown(LayerStatus &status) { this->template call(&Layer::shutdown, status); }

    virtual void handleHalt(LayerStatus &status) {  this->template call(&Layer::halt, status); }
    virtual void handleRecover(LayerStatus &status) { this->template call<LayerStatus::Warn>(&Layer::recover, status); }
public:
    LayerGroup(const std::string &n) : Layer(n) {}
};

class LayerStack : public LayerGroup<>{
    
protected:
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) { call_or_fail_rev(&Layer::write, &Layer::halt, status);}
    virtual void handleShutdown(LayerStatus &status) { call_rev(&Layer::shutdown, status); }
public:
    LayerStack(const std::string &n) : LayerGroup(n) {}
};

template<typename T> class LayerGroupNoDiag : public LayerGroup<T>{
public:
    LayerGroupNoDiag(const std::string &n) : LayerGroup<T>(n) {}
    virtual void handleDiag(LayerReport &report) {}
};

template<typename T> class DiagGroup : public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report);
    }
};



} // namespace canopen

#endif
