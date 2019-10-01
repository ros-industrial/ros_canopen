#ifndef H_CAN_DISPATCHER
#define H_CAN_DISPATCHER

#include <functional>
#include <memory>
#include <list>
#include <unordered_map>

#include <socketcan_interface/interface.h>
#include <boost/thread/mutex.hpp>

namespace can{

template< typename Listener > class SimpleDispatcher{
public:
    using Callable = typename Listener::Callable;
    using Type = typename Listener::Type;
    using ListenerConstSharedPtr = typename Listener::ListenerConstSharedPtr;
protected:
    class DispatcherBase;
    using DispatcherBaseSharedPtr = std::shared_ptr<DispatcherBase>;
    class DispatcherBase {
        DispatcherBase(const DispatcherBase&) = delete; // prevent copies
        DispatcherBase& operator=(const DispatcherBase&) = delete;

        class GuardedListener: public Listener{
            std::weak_ptr<DispatcherBase> guard_;
        public:
            GuardedListener(DispatcherBaseSharedPtr g, const Callable &callable): Listener(callable), guard_(g){}
            virtual ~GuardedListener() {
                DispatcherBaseSharedPtr d = guard_.lock();
                if(d){
                    d->remove(this);
                }
            }
        };

        boost::mutex &mutex_;
        std::list<const Listener* > listeners_;
    public:
        DispatcherBase(boost::mutex &mutex) : mutex_(mutex) {}
        void dispatch_nolock(const Type &obj) const{
           for(typename std::list<const Listener* >::const_iterator it=listeners_.begin(); it != listeners_.end(); ++it){
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

        static ListenerConstSharedPtr createListener(DispatcherBaseSharedPtr dispatcher, const  Callable &callable){
            ListenerConstSharedPtr l(new GuardedListener(dispatcher,callable));
            dispatcher->listeners_.push_back(l.get());
            return l;
        }
    };
    boost::mutex mutex_;
    DispatcherBaseSharedPtr dispatcher_;
public:
    SimpleDispatcher() : dispatcher_(new DispatcherBase(mutex_)) {}
    ListenerConstSharedPtr createListener(const Callable &callable){
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

template<typename K, typename Listener, typename Hash = std::hash<K> > class FilteredDispatcher: public SimpleDispatcher<Listener>{
    using BaseClass = SimpleDispatcher<Listener>;
    std::unordered_map<K, typename BaseClass::DispatcherBaseSharedPtr, Hash> filtered_;
public:
    using BaseClass::createListener;
    typename BaseClass::ListenerConstSharedPtr createListener(const K &key, const typename BaseClass::Callable &callable){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        typename BaseClass::DispatcherBaseSharedPtr &ptr = filtered_[key];
        if(!ptr) ptr.reset(new typename BaseClass::DispatcherBase(BaseClass::mutex_));
        return BaseClass::DispatcherBase::createListener(ptr, callable);
    }

    template <typename T>
    [[deprecated("provide key explicitly")]]
    typename BaseClass::ListenerConstSharedPtr createListener(const T &key, const typename BaseClass::Callable &callable){
        return createListener(static_cast<K>(key), callable);
    }

    void dispatch(const K &key, const typename BaseClass::Type &obj){
        boost::mutex::scoped_lock lock(BaseClass::mutex_);
        typename BaseClass::DispatcherBaseSharedPtr &ptr = filtered_[key];
        if(ptr) ptr->dispatch_nolock(obj);
        BaseClass::dispatcher_->dispatch_nolock(obj);
    }

    [[deprecated("provide key explicitly")]]
    void dispatch(const typename BaseClass::Type &obj){
        return dispatch(static_cast<K>(obj), obj);
    }

    operator typename BaseClass::Callable() { return typename BaseClass::Callable(this,&FilteredDispatcher::dispatch); }
};

} // namespace can
#endif
