#ifndef H_CAN_DISPATCHER
#define H_CAN_DISPATCHER

#include <socketcan_interface/interface.h>
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>

namespace can{

template< typename Listener > class SimpleDispatcher{
public:
    typedef typename Listener::Callable Callable;
    typedef typename Listener::Type Type;
protected:
    class DispatcherBase : boost::noncopyable{
        class GuardedListener: public Listener{
            boost::weak_ptr<DispatcherBase> guard_;
        public:
            GuardedListener(boost::shared_ptr<DispatcherBase> g, const Callable &callable): Listener(callable), guard_(g){}
            virtual ~GuardedListener() {
                boost::shared_ptr<DispatcherBase> d = guard_.lock();
                if(d){
                    d->remove(this);
                }
            }
        };
        
        boost::mutex &mutex_;
        std::list< Listener* > listeners_;
    public:
        DispatcherBase(boost::mutex &mutex) : mutex_(mutex) {}
        void dispatch_nolock(const Type &obj) const{
           for(typename std::list<Listener* >::const_iterator it=listeners_.begin(); it != listeners_.end(); ++it){
               (**it)(obj);
            }
        }
        void remove(Listener *d){
            boost::mutex::scoped_lock lock(mutex_);
            listeners_.remove(d);
        }
        size_t numListeners(){
            boost::mutex::scoped_lock lock(mutex_);
            return listeners_.size();
        }

        static typename Listener::Ptr createListener(boost::shared_ptr<DispatcherBase> dispatcher, const  Callable &callable){
            boost::shared_ptr<Listener > l(new GuardedListener(dispatcher,callable));
            dispatcher->listeners_.push_back(l.get());
            return l;
        }
    };
    boost::mutex mutex_;
    boost::shared_ptr<DispatcherBase> dispatcher_;
public:
    SimpleDispatcher() : dispatcher_(new DispatcherBase(mutex_)) {}
    typename Listener::Ptr createListener(const Callable &callable){
        boost::mutex::scoped_lock lock(mutex_);
        return DispatcherBase::createListener(dispatcher_, callable);
    }
    void dispatch(const Type &obj){
        boost::mutex::scoped_lock lock(mutex_);
        dispatcher_->dispatch_nolock(obj);
    }
    size_t numListeners(){
        return dispatcher_->numListeners();
    }
    operator Callable() { return Callable(this,&SimpleDispatcher::dispatch); }
};

template<typename K, typename Listener, typename Hash = boost::hash<K> > class FilteredDispatcher: public SimpleDispatcher<Listener>{
    typedef SimpleDispatcher<Listener> BaseClass;
    boost::unordered_map<K, boost::shared_ptr<typename BaseClass::DispatcherBase >, Hash> filtered_;
public:
    using BaseClass::createListener;
    typename Listener::Ptr createListener(const K &key, const typename BaseClass::Callable &callable){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        boost::shared_ptr<typename BaseClass::DispatcherBase > &ptr = filtered_[key];
        if(!ptr) ptr.reset(new typename BaseClass::DispatcherBase(BaseClass::mutex_));
        return BaseClass::DispatcherBase::createListener(ptr, callable);
    }
    void dispatch(const typename BaseClass::Type &obj){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        boost::shared_ptr<typename BaseClass::DispatcherBase > &ptr = filtered_[obj];
        if(ptr) ptr->dispatch_nolock(obj);
        BaseClass::dispatcher_->dispatch_nolock(obj);
    }
    operator typename BaseClass::Callable() { return typename BaseClass::Callable(this,&FilteredDispatcher::dispatch); }
};


template <typename T>class DispatchedInterface: public CommInterface, public StateInterface, public T{
    typedef FilteredDispatcher<const unsigned int, CommInterface::FrameListener> FrameDispatcher;
    typedef SimpleDispatcher<StateInterface::StateListener> StateDispatcher;
    FrameDispatcher frame_dispatcher_;
    StateDispatcher state_dispatcher_;
public:
    DispatchedInterface(): T(FrameDelegate(&frame_dispatcher_, &FrameDispatcher::dispatch),StateDelegate(&state_dispatcher_, &StateDispatcher::dispatch)) {}
    template<typename T1> DispatchedInterface(const T1 &t1): T(FrameDelegate(&frame_dispatcher_, &FrameDispatcher::dispatch),StateDelegate(&state_dispatcher_, &StateDispatcher::dispatch), t1) {}
    template<typename T1,typename T2> DispatchedInterface(const T1 &t1, const T2 &t2): T(FrameDelegate(&frame_dispatcher_, &FrameDispatcher::dispatch),StateDelegate(&state_dispatcher_, &StateDispatcher::dispatch), t1, t2) {}
    
    virtual bool send(const Frame & msg){
        return T::send(msg);
    }

    virtual FrameListener::Ptr createMsgListener(const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(delegate);
    }
    virtual FrameListener::Ptr createMsgListener(const Frame::Header&h , const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(h, delegate);
    }
    virtual StateListener::Ptr createStateListener(const StateDelegate &delegate){
        return state_dispatcher_.createListener(delegate);
    }
};


} // namespace can
#endif
