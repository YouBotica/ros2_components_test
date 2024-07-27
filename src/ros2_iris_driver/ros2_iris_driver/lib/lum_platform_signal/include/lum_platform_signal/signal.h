// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_SIGNAL_SIGNAL_H
#define LUM_PLATFORM_SIGNAL_SIGNAL_H

// Signal handlers are a C-level interface, and cannot perform actions that may throw.  So condition
// variables and mutexes cannot be used directly.  The workaround is to start a thread that monitors
// the simple variables used, and act when they are set.
//
// This code centralizes runtime signal/exit management.  Because the middleware can run in adverse
// environments where signaling has different behaviors (looking at you Unity), we must expose both
// a single place to register signals for multuiple threads to act on it, as well as a state flag to
// inform some components if we are actually shutting down or not.
//
// We are assuming that we will not receive multiple signals before the polling duration of
// the checking thread.  The thread will still call, but the signal number we received will be
// overwritten.  We are allowing this, as currently all signals are interpreted as "exit", but that
// will need to be changed if we handle other signal cases.
//
// We are also assuming that other classes or threads or libraries that may register a signal
// handler after us will behave properly, and not simply discard our handler, breaking our handler.
//
// Currently, any function registered to get a signal is called on all signals, with the specific
// signal sent in the observable.

#include <lum_common_types_observable/observable.h>

namespace lum {
namespace platform {
namespace signal {

using SignalHandle = lum::common::types::observable::SubscriptionHandle<std::int32_t>;
using SignalCallback = std::function<void(std::int32_t)>;

/// @namespace lum::platform::signal OS signal handling utilities

/// @brief Register a signal handler with the runtime.
///
/// register a callback function to be called when a signal is received.  Signals are not
/// intercepted until the first handler is registered.
///
/// @note if all handles are dropped, the system does not unregister to signals.  But since the
/// signals are not propogated the signals are just ignored
///
/// @param handler [in] a callback function or lambda that is called when a signal is received
/// @return a subscription handle.  The function is not called unless the handle is kept in scope.
SignalHandle registerHandler(const SignalCallback& handler);

/// @brief utility function to see if we have registered a signal handler, and have been interrupted
/// @return true if we received a signal and should halt
bool wasInterrupted();

} // namespace signal
} // namespace platform
} // namespace lum

#endif
