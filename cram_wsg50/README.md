cram_wsg50
===

Example usage
---
Start C++ driver in console
```
roslaunch wsg_50_driver wsg_50_tcp.launch
```

Compile and change into package
```lisp
(asdf:load-system :cram-wsg50)
(in-package :cram-wsg50)
```

Get a handle
```lisp
(roslisp:start-ros-node "wsg50-example")
(defparameter *wsg50-handle* (make-wsg50-handle "wsg_50_driver"))
```

Close the gripper
```lisp
(move-wsg50 *wsg50-handle* 110 10 5)
(move-wsg50 *wsg50-handle* '(:pos 25 :speed 20 :force 10))
```

Get the status fluent
```lisp
(status-fluent *wsg50-handle*)
```
