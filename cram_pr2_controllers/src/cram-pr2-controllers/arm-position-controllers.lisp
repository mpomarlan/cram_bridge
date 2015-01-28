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

(defclass pr2-arm-position-controller-handle ()
  ((client :initarg :client :accessor client :type actionlib::action-client)
   (lock :initform (make-mutex :name (string (gensym "PR2-ARM-POSITION-CTRL-LOCK-")))
         :accessor lock :type mutex
         :documentation "For internal use. Mutex to guard pr2 arm controller.")
   (joint-names :initarg :joint-names :accessor joint-names :type list))
  (:documentation "Handle to standard PR2 position-resolved arm controllers."))

(define-condition pr2-arm-controller-error (simple-error) ())

;;; EXPORTED API

(defun make-pr2-arm-position-controller-handle (action-name joint-names)
  (declare (type string action-name)
           (type list joint-names))
  (make-instance
   'pr2-arm-position-controller-handle
   :client (actionlib:make-action-client
            action-name "pr2_controllers_msgs/JointTrajectoryAction")
   :joint-names joint-names))

(defun move-arm (handle goal-state execution-time)
  (declare (type pr2-arm-position-controller-handle handle)
           (type list goal-state)
           (type number execution-time))
  (with-recursive-lock ((lock handle))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait 
         (client handle)
         (actionlib:make-action-goal (client handle)
           :trajectory (make-trajectory-msg handle goal-state execution-time)))
      ;; TODO(Georg): think about a good low-level interface
      (declare (ignore status result)))))

;;; INTERNAL API

(defun make-trajectory-msg (handle goal-state execution-time)
  (declare (type pr2-arm-position-controller-handle handle)
           (type list goal-state)
           (type number execution-time))
  (roslisp:make-msg
   "trajectory_msgs/JointTrajectory"
   :header (roslisp:make-msg
            "std_msgs/Header"
            :stamp (roslisp:ros-time))
   :joint_names (coerce (mapcar 
                         (lambda (keyword) (string-downcase (string keyword)))
                         (joint-names handle))
                        'vector)
   :points (coerce `(,(make-trajectory-point handle goal-state execution-time)) 'vector)))

(defun make-trajectory-point (handle goal-state execution-time)
  (declare (type pr2-arm-position-controller-handle handle)
           (type list goal-state)
           (type number execution-time))
  (let ((goal-configuration
          (mapcar (lambda (joint)
                    (getf (getf goal-state joint) :position))
                  (joint-names handle))))
    (roslisp:make-msg 
     "trajectory_msgs/JointTrajectoryPoint"
     :positions (coerce goal-configuration 'vector)
     :time_from_start execution-time)))