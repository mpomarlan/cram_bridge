cram_wsg50
===

Example usage
---

```
(start-ros-node "wsg50-example")
(defparameter *wsg50-handle* (make-wsg50-handle "wsg_50_driver")
(move-wsg50 *wsg50-handle* *wsg50-closed-width* 10 1)
```
