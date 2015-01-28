;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(defclass pr2-controller-manager-handle ()
  ((client :initarg :client :accessor client :type roslisp:persistent-service
           :documentation "For internal use. Persistent service to controller manager.")
   (lock :initform (make-mutex :name (string (gensym "PR2-CONTROLLER-MANAGER-LOCK-")))
         :accessor lock :type mutex
         :documentation "For internal use. Mutex to guard pr2 controller manager."))
  (:documentation "Handle to PR2 controller manager."))

(defparameter *pr2-controller-manager-ns* "pr2_controller_manager")

(defparameter *pr2-velocity-controllers*
  (let ((vel-controllers (make-hash-table)))
    (setf (gethash 'left vel-controllers) 
          '("l_shoulder_pan_velocity_controller"
            "l_shoulder_lift_velocity_controller"
            "l_upper_arm_roll_velocity_controller"
            "l_elbow_flex_velocity_controller"
            "l_forearm_roll_velocity_controller"
            "l_wrist_flex_velocity_controller"
            "l_wrist_roll_velocity_controller"))
    (setf (gethash 'right vel-controllers) 
          '("r_shoulder_pan_velocity_controller"
            "r_shoulder_lift_velocity_controller"
            "r_upper_arm_roll_velocity_controller"
            "r_elbow_flex_velocity_controller"
            "r_forearm_roll_velocity_controller"
            "r_wrist_flex_velocity_controller"
            "r_wrist_roll_velocity_controller"))
    (setf (gethash 'all vel-controllers) 
          '("l_shoulder_pan_velocity_controller"
            "l_shoulder_lift_velocity_controller"
            "l_upper_arm_roll_velocity_controller"
            "l_elbow_flex_velocity_controller"
            "l_forearm_roll_velocity_controller"
            "l_wrist_flex_velocity_controller"
            "l_wrist_roll_velocity_controller"
            "r_shoulder_pan_velocity_controller"
            "r_shoulder_lift_velocity_controller"
            "r_upper_arm_roll_velocity_controller"
            "r_elbow_flex_velocity_controller"
            "r_forearm_roll_velocity_controller"
            "r_wrist_flex_velocity_controller"
            "r_wrist_roll_velocity_controller"))
    vel-controllers))
 
(defparameter *pr2-position-controllers*
  (let ((vel-controllers (make-hash-table)))
    (setf (gethash 'left vel-controllers) '("l_arm_controller"))
    (setf (gethash 'right vel-controllers) '("r_arm_controller"))
    (setf (gethash 'all vel-controllers) '("l_arm_controller" "r_arm_controller"))
    vel-controllers))

(defun make-pr2-controller-manager-handle (&key (namespace *pr2-controller-manager-ns*))
  (make-instance 
   'pr2-controller-manager-handle
   :client (make-instance 
            'roslisp:persistent-service
            :service-name (concatenate 'string namespace "/switch_controller")
            :service-type "pr2_mechanism_msgs/SwitchController")))

(defun cleanup-pr2-controller-manager-handle (handle)
  (declare (type pr2-controller-manager-handle handle))
  (with-recursive-lock ((lock handle))
    (when (client handle) (roslisp:close-persistent-service (client handle)))))

(define-condition switch-controller-error (simple-error) ())

(defun switch-controllers (handle start stop)
  "Calls the switch_controller service in pr2-controller-manager-handle `handle' starting
 all controllers in sequence `start' and stopping all controllers in sequence `stop'.
 Controllers are refered to by their names."
  (declare (type pr2-controller-manager-handle handle)
           (type sequence start stop))
  (roslisp:with-fields (ok)
      (with-recursive-lock ((lock handle))
        (roslisp:call-persistent-service
         (client handle)
         :start_controllers (map 'vector #'identity start)
         :stop_controllers (map 'vector #'identity stop)
         :strictness (roslisp-msg-protocol:symbol-code
                      'pr2_mechanism_msgs-srv:switchcontroller-request
                      :strict)))
    (when (eql ok 0)
      (error 'switch-controller
             :format-control "Switching controllers failed. Start: ~a, stop: ~a."
             :format-arguments (list start stop)))
    t))

(defun ensure-vel-controllers (handle &key (arms 'all))
  "Uses pr2-controller-manager-handle `handle' to make sure `arms' are running
 velocity-resolved controllers."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers handle vel-ctrls pos-ctrls)))

(defun ensure-pos-controllers (handle &key (arms 'all))
  "Uses pr2-controller-manager-handle `handle' to make sure `arms' are running
 position-resolved controllers."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers handle pos-ctrls vel-ctrls)))

(defun stop-controllers (handle &key (arms 'all))
  "Uses pr2-controller-manager-handle `handle' to stop all controllers for `arm'."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers handle nil (concatenate 'list pos-ctrls vel-ctrls))))

(defun get-velocity-controller-names (arms)
  "Returns list of names of velocity-resolved controllers for symbol `arms'."
  (multiple-value-bind (controllers controllers-present)
      (gethash arms *pr2-velocity-controllers*)
    (unless controllers-present
      (error 'switch-controller
             :format-control "Could not lookup pr2 velocity controllers for: ~a."
             :format-arguments (list arms)))
    controllers))

(defun get-position-controller-names (arms)
  "Returns list of names of position-resolved controllers for symbol `arms'."
  (multiple-value-bind (controllers controllers-present)
      (gethash arms *pr2-position-controllers*)
    (unless controllers-present
      (error 'switch-controller
             :format-control "Could not lookup pr2 position controllers for: ~a."
             :format-arguments (list arms)))
    controllers))