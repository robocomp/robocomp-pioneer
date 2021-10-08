//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `Monitor.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __Monitor_h__
#define __Monitor_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompMonitor
{

class Monitor;
class MonitorPrx;

}

namespace RoboCompMonitor
{

class HardwareFailedException : public ::Ice::UserExceptionHelper<HardwareFailedException, ::Ice::UserException>
{
public:

    virtual ~HardwareFailedException();

    HardwareFailedException(const HardwareFailedException&) = default;

    HardwareFailedException() = default;

    /**
     * One-shot constructor to initialize all data members.
     */
    HardwareFailedException(const ::std::string& what) :
        what(::std::move(what))
    {
    }

    /**
     * Obtains a tuple containing all of the exception's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&> ice_tuple() const
    {
        return std::tie(what);
    }

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    ::std::string what;
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

enum class MonitorStates : unsigned char
{
    Alive,
    Fail,
    Disconnected
};

}

namespace RoboCompMonitor
{

class Monitor : public virtual ::Ice::Object
{
public:

    using ProxyType = MonitorPrx;

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(::std::string id, const ::Ice::Current& current) const override;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current& current) const override;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual ::std::string ice_id(const ::Ice::Current& current) const override;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual MonitorStates getState(const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_getState(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompMonitor
{

class MonitorPrx : public virtual ::Ice::Proxy<MonitorPrx, ::Ice::ObjectPrx>
{
public:

    MonitorStates getState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompMonitor::MonitorStates>(true, this, &MonitorPrx::_iceI_getState, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getStateAsync(const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompMonitor::MonitorStates>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompMonitor::MonitorStates, P>(false, this, &MonitorPrx::_iceI_getState, context);
    }

    ::std::function<void()>
    getStateAsync(::std::function<void(::RoboCompMonitor::MonitorStates)> response,
                  ::std::function<void(::std::exception_ptr)> ex = nullptr,
                  ::std::function<void(bool)> sent = nullptr,
                  const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompMonitor::MonitorStates>(response, ex, sent, this, &RoboCompMonitor::MonitorPrx::_iceI_getState, context);
    }

    /// \cond INTERNAL
    void _iceI_getState(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompMonitor::MonitorStates>>&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    MonitorPrx() = default;
    friend ::std::shared_ptr<MonitorPrx> IceInternal::createProxy<MonitorPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<typename S>
struct StreamReader<::RoboCompMonitor::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompMonitor::HardwareFailedException& v)
    {
        istr->readAll(v.what);
    }
};

template<>
struct StreamableTraits< ::RoboCompMonitor::MonitorStates>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 2;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompMonitor
{

using MonitorPtr = ::std::shared_ptr<Monitor>;
using MonitorPrxPtr = ::std::shared_ptr<MonitorPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompMonitor
{

class Monitor;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< Monitor>&);
::IceProxy::Ice::Object* upCast(Monitor*);
/// \endcond

}

}

namespace RoboCompMonitor
{

class Monitor;
/// \cond INTERNAL
::Ice::Object* upCast(Monitor*);
/// \endcond
typedef ::IceInternal::Handle< Monitor> MonitorPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompMonitor::Monitor> MonitorPrx;
typedef MonitorPrx MonitorPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(MonitorPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompMonitor
{

class HardwareFailedException : public ::Ice::UserException
{
public:

    HardwareFailedException() {}
    /**
     * One-shot constructor to initialize all data members.
     */
    explicit HardwareFailedException(const ::std::string& what);
    virtual ~HardwareFailedException() throw();

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    virtual ::std::string ice_id() const;
    /**
     * Polymporphically clones this exception.
     * @return A shallow copy of this exception.
     */
    virtual HardwareFailedException* ice_clone() const;
    /**
     * Throws this exception.
     */
    virtual void ice_throw() const;

    ::std::string what;

protected:

    /// \cond STREAM
    virtual void _writeImpl(::Ice::OutputStream*) const;
    virtual void _readImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

enum MonitorStates
{
    Alive,
    Fail,
    Disconnected
};

}

namespace RoboCompMonitor
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompMonitor::Monitor::begin_getState.
 * Create a wrapper instance by calling ::RoboCompMonitor::newCallback_Monitor_getState.
 */
class Callback_Monitor_getState_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Monitor_getState_Base> Callback_Monitor_getStatePtr;

}

namespace IceProxy
{

namespace RoboCompMonitor
{

class Monitor : public virtual ::Ice::Proxy<Monitor, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompMonitor::MonitorStates getState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getState(_iceI_begin_getState(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getState(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getState(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getState(const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getState(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getState(const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getState(context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getState(const ::RoboCompMonitor::Callback_Monitor_getStatePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getState(::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_getState(const ::Ice::Context& context, const ::RoboCompMonitor::Callback_Monitor_getStatePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getState(context, cb, cookie);
    }

    ::RoboCompMonitor::MonitorStates end_getState(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getState(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    /**
     * Obtains the Slice type ID corresponding to this interface.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:
    /// \cond INTERNAL

    virtual ::IceProxy::Ice::Object* _newInstance() const;
    /// \endcond
};

}

}

namespace RoboCompMonitor
{

class Monitor : public virtual ::Ice::Object
{
public:

    typedef MonitorPrx ProxyType;
    typedef MonitorPtr PointerType;

    virtual ~Monitor();

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(const ::std::string& id, const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual const ::std::string& ice_id(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual MonitorStates getState(const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_getState(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

protected:

    /// \cond STREAM
    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
inline bool operator==(const Monitor& lhs, const Monitor& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const Monitor& lhs, const Monitor& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompMonitor::HardwareFailedException>
{
    static const StreamHelperCategory helper = StreamHelperCategoryUserException;
};

template<typename S>
struct StreamWriter< ::RoboCompMonitor::HardwareFailedException, S>
{
    static void write(S* ostr, const ::RoboCompMonitor::HardwareFailedException& v)
    {
        ostr->write(v.what);
    }
};

template<typename S>
struct StreamReader< ::RoboCompMonitor::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompMonitor::HardwareFailedException& v)
    {
        istr->read(v.what);
    }
};

template<>
struct StreamableTraits< ::RoboCompMonitor::MonitorStates>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 2;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

}
/// \endcond

namespace RoboCompMonitor
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompMonitor::Monitor::begin_getState.
 * Create a wrapper instance by calling ::RoboCompMonitor::newCallback_Monitor_getState.
 */
template<class T>
class CallbackNC_Monitor_getState : public Callback_Monitor_getState_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(MonitorStates);

    CallbackNC_Monitor_getState(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        MonitorPrx proxy = MonitorPrx::uncheckedCast(result->getProxy());
        MonitorStates ret;
        try
        {
            ret = proxy->end_getState(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret);
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompMonitor::Monitor::begin_getState.
 */
template<class T> Callback_Monitor_getStatePtr
newCallback_Monitor_getState(const IceUtil::Handle<T>& instance, void (T::*cb)(MonitorStates), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Monitor_getState<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompMonitor::Monitor::begin_getState.
 */
template<class T> Callback_Monitor_getStatePtr
newCallback_Monitor_getState(T* instance, void (T::*cb)(MonitorStates), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Monitor_getState<T>(instance, cb, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompMonitor::Monitor::begin_getState.
 * Create a wrapper instance by calling ::RoboCompMonitor::newCallback_Monitor_getState.
 */
template<class T, typename CT>
class Callback_Monitor_getState : public Callback_Monitor_getState_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(MonitorStates, const CT&);

    Callback_Monitor_getState(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        MonitorPrx proxy = MonitorPrx::uncheckedCast(result->getProxy());
        MonitorStates ret;
        try
        {
            ret = proxy->end_getState(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, CT::dynamicCast(result->getCookie()));
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompMonitor::Monitor::begin_getState.
 */
template<class T, typename CT> Callback_Monitor_getStatePtr
newCallback_Monitor_getState(const IceUtil::Handle<T>& instance, void (T::*cb)(MonitorStates, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Monitor_getState<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompMonitor::Monitor::begin_getState.
 */
template<class T, typename CT> Callback_Monitor_getStatePtr
newCallback_Monitor_getState(T* instance, void (T::*cb)(MonitorStates, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Monitor_getState<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif