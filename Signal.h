#ifndef SIGNAL_H
#define SIGNAL_H

#include <functional>
#include <map>
#include <mutex>

// A signal object may call multiple slots with the
// same signature. You can connect functions to the signal
// which will be called when the emit() method on the
// signal object is invoked. Any argument passed to emit()
// will be passed to the given functions.

template <typename Args>
class Signal {

 public:

  Signal() : current_id_(0), disconnect_later(false) {}

  // copy creates new signal
  Signal(Signal const& other) : current_id_(0), disconnect_later(false) {}

  // connects a member function to this Signal
  template <typename T>
  int connect_member(T *inst, void (T::*func)(Args)) {
    return connect([=](Args args) { 
      (inst->*func)(args); 
    });
  }

  // connects a const member function to this Signal
  template <typename T>
  int connect_member(T *inst, void (T::*func)(Args) const) {
    return connect([=](Args args) {
      (inst->*func)(args); 
    });
  }

  // connects a std::function to the signal. The returned
  // value can be used to disconnect the function again
  int connect(std::function<void(Args)> const& slot) const {
	std::lock_guard<std::mutex> lock(signal_mutex_);
    slots_.insert(std::make_pair(++current_id_, slot));
    return current_id_;
  }

  // disconnects a previously connected function
  void disconnect(int id) const {
	// std::lock_guard<std::mutex> lock(signal_mutex_);
	std::unique_lock<std::mutex> ulock(signal_mutex_, std::try_to_lock);
	if (ulock.owns_lock())
	{
		slots_.erase(id);
	}
	else
	{
		disconnect_later = true;
	}
  }

  // disconnects all previously connected functions
  void disconnect_all() const {
	std::lock_guard<std::mutex> lock(signal_mutex_);
	slots_.clear();
  }

  // calls all connected functions
  void emit(Args p) {
	std::lock_guard<std::mutex> lock(signal_mutex_);
    for(auto it : slots_) {	  
      it.second(p);
    }
  }

  // assignment creates new Signal
  Signal& operator=(Signal const& other) {
    disconnect_all();
  }

 private:
  mutable std::map<int, std::function<void(Args)>> slots_;
  mutable int current_id_;
  mutable std::mutex signal_mutex_;
  bool disconnect_later;
};

#endif /* SIGNAL_H */