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

(defparameter *pr2-velocity-controllers*
  (let ((vel-controllers (make-hash-table)))
    (setf (gethash 'left vel-controllers) '("l_arm_vel"))
    (setf (gethash 'right vel-controllers) '("r_arm_vel"))
    (setf (gethash 'all vel-controllers) '("l_arm_vel" "r_arm_vel"))
    vel-controllers))

(defparameter *pr2-position-controllers*
  (let ((vel-controllers (make-hash-table)))
    (setf (gethash 'left vel-controllers) '("l_arm_controller"))
    (setf (gethash 'right vel-controllers) '("r_arm_controller"))
    (setf (gethash 'all vel-controllers) '("l_arm_controller" "r_arm_controller"))
    vel-controllers))

(defparameter *pr2-controller-manager-ns* "pr2_controller_manager")

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

(define-condition switch-controller-error (simple-error) ())

(defun switch-controllers (start stop &key (namespace *pr2-controller-manager-ns*))
  "Calls the switch_controller service in `namespace' starting all controllers in
list `start' and stopping all controllers in list `stop'. Controllers are refered
to by their names."
  (declare (type sequence start stop)
           (type string namespace))
  (roslisp:with-fields (ok)
      (roslisp:call-service
       (concatenate 'string namespace "/switch_controller")
       "pr2_mechanism_msgs/SwitchController"
       :start_controllers (map 'vector #'identity start)
       :stop_controllers (map 'vector #'identity stop)
       :strictness (roslisp-msg-protocol:symbol-code
                    'pr2_mechanism_msgs-srv:switchcontroller-request
                    :strict))
    (when (eql ok 0)
      (error 'switch-controller
             :format-control "Switching controllers failed. Start: ~a, stop: ~a."
             :format-arguments (list start stop)))
    t))

(defun ensure-vel-controllers (&key (arms 'all))
  "Makes sure `arms' are running velocity-resolved controllers."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers vel-ctrls pos-ctrls)))

(defun ensure-pos-controllers (&key (arms 'all))
  "Makes sure `arms' are running position-resolved controllers."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers pos-ctrls vel-ctrls)))

(defun stop-controllers (&key (arms 'all))
  "Stops all controllers for `arm'."
  (let ((pos-ctrls (get-position-controller-names arms))
        (vel-ctrls (get-velocity-controller-names arms)))
    (switch-controllers nil (concatenate 'list pos-ctrls vel-ctrls))))