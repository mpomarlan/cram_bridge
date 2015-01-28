cram_wsg50
===

Example usage
---

Get a handle
```
(start-ros-node "wsg50-example")
(defparameter *wsg50-handle* (make-wsg50-handle "wsg_50_driver")
```

Close the gripper
```
(move-wsg50 *wsg50-handle* *wsg50-closed-width* 10 1)
```

Get the status fluent
```
(status-fluent *wsg50-handle*)
```
