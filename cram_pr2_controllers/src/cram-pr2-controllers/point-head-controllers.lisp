;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to
;;; endorse or promote products derived from this software without specific
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :pr2-controllers)

(defstruct pr2-point-head-handle 
  "Handle to standard PR2 point head controllers."
  (client nil :read-only nil)
  (action-name 
    "/head_traj_controller/point_head_action" :type string :read-only t)
  (action-type
   "pr2_controllers_msgs/PointHeadAction" :type string :read-only t)
  (lock 
   (make-mutex :name (string (gensym "PR2-POINT-HEAD-LOCK-")))
   :type mutex :read-only nil))

(defun point-head (handle pose-stamped &optional (max-velocity 10) (min-duration 0.3)
                                         (execute-timeout 0) (preempt-timeout 0))
  "Uses `handle' to point the head of the PR2 towards `pose-stamped'. `max-velocity',
 and `min-duration' are parameters to tweak the speed of motion. `execute-timeout'
 specifies the time (in seconds) after which the client automatically cancels the
 goal `preempt-timeout' specifies the time (in seconds) for which the client
 waits to hear from the server after cancellation. A timeout of 0 seconds means infinity."
  (declare (type pr2-point-head-handle handle)
           (type cl-tf:pose-stamped pose-stamped))
  (with-slots (client lock) handle
    (with-recursive-lock (lock)
      (ensure-point-head-client handle)
      (actionlib-lisp:send-goal-and-wait
       client 
       (make-point-head-goal handle pose-stamped max-velocity min-duration)
       execute-timeout preempt-timeout))))

;;;
;;; INTERNAL API
;;;

(defun ensure-point-head-client (handle)
  "Makes sure that the action-client inside `handle' is not nil."
  (declare (type pr2-point-head-handle handle))
  (with-slots (client action-name action-type lock) handle
    (with-recursive-lock (lock)
      (unless client
        (setf client (actionlib-lisp:make-simple-action-client action-name action-type))))))

(defun make-point-head-goal (handle pose-stamped max-velocity min-duration)
  "Creates and returns a point-head goal message from the given parameters."
  (declare (type pr2-point-head-handle handle)
           (type cl-tf:pose-stamped pose-stamped)
           (type number max-velocity min-duration))
  (with-slots (client lock) handle
    (with-recursive-lock (lock)
      (ensure-point-head-client handle)
      (actionlib-lisp:make-action-goal-msg client
        max_velocity max-velocity
        min_duration min-duration
        pointing_frame "/high_def_frame"
        (x pointing_axis) 1.0
        (y pointing_axis) 0.0
        (z pointing_axis) 0.0
        target (pose-stamped->point-stamped-msg pose-stamped)))))

(defun pose-stamped->point-stamped-msg (ps)
  "Returns a message of type geometry_msgs/PointStamped which matches the
 PoseStamped `ps'."
  (roslisp:make-message
   "geometry_msgs/PointStamped"
   (stamp header) (tf:stamp ps)
   (frame_id header) (tf:frame-id ps)
   (x point) (cl-transforms:x
              (cl-transforms:origin ps))
   (y point) (cl-transforms:y
              (cl-transforms:origin ps))
   (z point) (cl-transforms:z
              (cl-transforms:origin ps))))