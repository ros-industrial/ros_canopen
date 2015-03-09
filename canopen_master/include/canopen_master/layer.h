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

    const void warn(const std::string & r = "") { set(WARN, r); }
    const void error(const std::string & r = "") { set(ERROR, r); }
    const void stale(const std::string & r = "") { set(STALE, r); }
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

    virtual void pending(LayerStatus &status) = 0;
    virtual void read(LayerStatus &status) = 0;
    virtual void write(LayerStatus &status) = 0;
    
    virtual void diag(LayerReport &report) = 0;
    
    virtual void init(LayerStatus &status) = 0;
    virtual void shutdown(LayerStatus &status) = 0;
    
    virtual void halt(LayerStatus &status) = 0;
    virtual void recover(LayerStatus &status) = 0;
    
    Layer(const std::string &n) : name(n) {}
    
    virtual ~Layer() {}
};

template<typename T> class VectorHelper{
protected:
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    
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
    void destroy() { layers.clear(); }
public:
    void add(const boost::shared_ptr<T> &l) { layers.push_back(l); }
};

template<typename T=Layer> class LayerVector : public Layer, public VectorHelper<T> {
protected:
    typedef typename VectorHelper<T>::vector_type vector_type;
    typedef typename vector_type::iterator iterator;
    typedef typename vector_type::reverse_iterator reverse_iterator;

    class Iterator{
    public:
        Iterator &operator=(const iterator& it){
            boost::mutex::scoped_lock lock(mutex_);
            it_ = it;
            return *this;
        }
        operator iterator() {
            boost::mutex::scoped_lock lock(mutex_);
            return it_;
        }
        iterator swap(const iterator& it){
            boost::mutex::scoped_lock lock(mutex_);
            iterator old = it_;
            it_ = it;
            return old;
        }
    private:
        boost::mutex mutex_;
        iterator it_;
    } pending_;


    void bringup(void(Layer::*func)(LayerStatus&), void(Layer::*func_fail)(LayerStatus&), LayerStatus &status){
        iterator it = this->layers.begin();
        for(; it != this->layers.end(); ++it){
            pending_ = it;
            ((**it).*func)(status);
            if(!status.bounded<LayerStatus::Warn>()) break;
        }
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(func_fail, omit, reverse_iterator(it), this->layers.rend());
        }
        pending_ = it;
    }

    void callFwdOrFail (void(Layer::*func)(LayerStatus&), void(Layer::*func_fail)(LayerStatus&), LayerStatus &status){
        iterator end = pending_;
        iterator it = this->template call<LayerStatus::Warn>(func, status, this->layers.begin(), end);
        if(it != end){
            LayerStatus omit;
            this->template call(func_fail, omit, this->layers.rbegin(), reverse_iterator(it));
            omit.error();
            this->template call(func, omit, it+1, end);
        }
    }
    LayerVector(const std::string &n) : Layer(n) {}
public:
    virtual void read(LayerStatus &status){
        callFwdOrFail(&Layer::read, &Layer::halt, status);
    }
    virtual void pending(LayerStatus &status){
        iterator end = pending_;
        if(end != this->layers.end()){
            (**end).pending(status);
        }
    }
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report, this->layers.begin(), (iterator) pending_);
    }
    virtual void init(LayerStatus &status) {
        bringup(&Layer::init, &Layer::shutdown, status);
    }
    virtual void recover(LayerStatus &status) {
        bringup(&Layer::recover, &Layer::halt, status);
    }
};

class LayerStack : public LayerVector<>{
    
public:
    virtual void write(LayerStatus &status){
        reverse_iterator begin = reverse_iterator(pending_);

        reverse_iterator it = call<LayerStatus::Warn>(&Layer::write, status, begin, layers.rend());
        if(it != layers.rend()){
            LayerStatus omit;
            call(&Layer::halt, omit, begin, reverse_iterator(it));
            omit.error();
            call(&Layer::write, omit, it+1, layers.rend());
        }
    }
    virtual void shutdown(LayerStatus &status){
        reverse_iterator begin = reverse_iterator(pending_.swap(layers.begin()));
        if( begin != layers.rbegin()) --begin; // include pendign layer
        call(&Layer::shutdown, status, begin, layers.rend());
    }
    virtual void halt(LayerStatus &status){
        reverse_iterator begin = reverse_iterator(pending_);
        if( begin != layers.rbegin()) --begin; // include pendign layer
        call(&Layer::halt, status, begin, layers.rend());
    }

    LayerStack(const std::string &n) : LayerVector(n) {}
};

template<typename T> class LayerGroup : public LayerVector<T>{
    typedef LayerVector<T> V;

public:
    virtual void write(LayerStatus &status){
        this->callFwdOrFail(&Layer::write, &Layer::halt, status);
    }
    virtual void shutdown(LayerStatus &status){
        typename V::iterator begin = this->pending_.swap(this->layers.begin());
        if(begin != this->layers.begin()) --begin; // include pendign layer
        this->template call(&Layer::shutdown, status, begin, this->layers.end());
    }
    virtual void halt(LayerStatus &status){
        typename V::iterator begin = this->pending_;
        if( begin != this->layers.begin()) --begin; // include pendign layer
        this->template call(&Layer::halt, status, begin, this->layers.end());
    }
    LayerGroup(const std::string &n) : LayerVector<T>(n) {}
};

template<typename T> class LayerGroupNoDiag : public LayerGroup<T>{
public:
    LayerGroupNoDiag(const std::string &n) : LayerGroup<T>(n) {}
    virtual void diag(LayerReport &report){
        // no report
    }
};

template<typename T> class DiagGroup : public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report, this->layers.begin(), this->layers.end());
    }
};



} // namespace canopen

#endif
