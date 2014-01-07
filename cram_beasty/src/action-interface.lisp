;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-beasty)

(define-condition beasty-command-error (error)
  ((text :initarg :text :reader text))
  (:documentation "Condition signalling an error while commanding Beasty controller."))

(defclass beasty-interface ()
  ((action-client :initarg :action-client :reader action-client 
                  :type actionlib::action-client
                  :documentation "For internal use. ROS action client to communicate with controller.")
   (session-id :initarg :session-id :accessor session-id :type number
               :documentation "For internal use. ID of current communication session.")
   (cmd-id :initarg :cmd-id :accessor cmd-id :type number
           :documentation "For internal use. cmd-id to be used in the next goal.")
   (state-sub :initform nil :accessor state-sub
              :documentation "For internal use. Subscriber listening to state-topic of server.")
   (state :initform (cram-language:make-fluent :value (make-instance 'beasty-state))
          :accessor state :type cram-language:value-fluent
          :documentation "Fluent with last state reported from beasty controller.")
   (robot :initform (make-instance 'beasty-robot) :accessor robot :type beasty-robot
          :documentation "Robot representation of LWR controlled by this interface."))
  (:documentation "Action-client interface with book-keeping for LWR controller Beasty."))

(defclass beasty-state ()
  ((motor-power-on :initarg :motor-power-on :reader motor-power-on :type boolean
                   :documentation "Indicates whether Beasty has all motors powered on.")
   (safety-released :initarg :safety-released :reader safety-released :type boolean
                    :documentation "Indicates whether safety buttons are released.")
   (joint-values :initarg :joint-values :reader joint-values
                 :type vector :documentation "Current joint values of LWR arm.")
   (tcp-pose :initarg :tcp-pose :reader tcp-pose :type cl-transforms:transform
              :documentation "Pose of tcp frame w.r.t. to arm base frame.")
   (joint-contacts :initarg :joint-contacts :reader joint-contacts
                   :type vector :documentation "Detected contacts per joint of LWR arm.")
   (joint-collisions :initarg :joint-collisions :reader joint-collisions
                     :type vector :documentation "Detected collisions per joint of LWR."))
  (:documentation "Representation of state reported from Beasty LWR controller."))

(defun make-beasty-interface (action-name)
  "Creates a BEASTY interface. `action-name' is the name of the action used to create
   the ROS action-client and will also be used to identify the BEASTY interface."
  (declare (type string action-name))
  (let ((action-client (actionlib:make-action-client action-name "dlr_msgs/RCUAction")))
    (actionlib:wait-for-server action-client 2.0)
    (multiple-value-bind (session-id cmd-id)
        (login-beasty action-client)
      (let ((interface (make-instance 'beasty-interface 
                                      :action-client action-client
                                      :session-id session-id
                                      :cmd-id cmd-id)))
        (add-state-subscriber interface action-name)
        interface))))

(defun command-beasty (interface parameters &optional (safety nil))
  "Sends a command to beasty controller behind `interface'. Users can alter motion command
 with `parameters' and `safety'."
  (declare (type beasty-interface interface))
  (let ((goal (actionlib:make-action-goal (action-client interface)
                :command (get-beasty-command-code 
                          (infer-command-symbol parameters))
                :parameters (make-parameter-msg interface parameters safety))))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait (action-client interface) goal)
      (unless (equal :succeeded status)
        (error 'beasty-command-error :test "Error commanding beasty action interface."))
      (with-fields (state) result
      (with-fields (com) state
        (with-fields (cmd_id) com
          (setf (cmd-id interface) (elt cmd_id (get-beasty-command-code
                                                (infer-command-symbol parameters))))))))))

(defun release-beasty-safety (interface)
  "Releases the software safety buttons of the beasty controller behind `interface'."
  (declare (type beasty-interface))
  (let ((reset-params (make-instance 'reset-safety-parameters)))
    (command-beasty interface reset-params nil)))

(defun safety-released-p (interface)
  "Checks whether the safety brakes of LWR arm behind `interface' are released."
  (declare (type beasty-interface interface))
  (safety-released (cram-language:value (state interface))))

(defun ensure-safety-released (interface)
  "Makes sure that the safety brakes of LWR arm behind `interface' are released."
  (unless (safety-released-p interface)
    (release-beasty-safety interface)))

(defun motors-on-p (interface)
  "Checks whether the motors of LWR arm behind `interface' are powered on."
  (declare (type cram-beasty::beasty-interface interface))
  (motor-power-on (cram-language:value (state interface))))

(defun add-state-subscriber (interface namespace)
  "Adds a beasty state-subscriber with topic `namespace'/state to `interface'. Said 
subscriber converts state-msg into an instance of class 'beasty-state' and saves it in the
'state' slot of `interface'."
  (declare (type beasty-interface interface)
           (type string namespace))
  (let ((subscriber 
          (roslisp:subscribe (concatenate 'string namespace "/state") 
                             "dlr_msgs/rcu2tcu"
                             (lambda (msg)
                               (setf (cram-language:value (state interface))
                                     (from-msg msg)))
                             :max-queue-length 1)))
    (setf (state-sub interface) subscriber)))

(defun infer-command-symbol (parameters)
  "Infers the RCUGoal-command type based on the type of `parameters'."
  (etypecase parameters
    (gravity-control-parameters :CHANGE_BEHAVIOUR)
    (joint-impedance-control-parameters :MOVETO)
    (cartesian-impedance-control-parameters :MOVETO)
    (reset-safety-parameters :RESET_SAFETY)))
                           
(defun make-parameter-msg (interface parameters &optional (safety nil))
  "Creates the appropriate parameter message to control `robot' behind `interface' to
   perform motion specified by `parameters' with `safety'."
  (declare (type beasty-interface interface)
           (ignore safety))
  (multiple-value-bind (robot-msg settings-msg) (to-msg (robot interface))
    (multiple-value-bind (controller-msg interpolator-msg) (to-msg parameters)
      (roslisp:make-msg 
       "dlr_msgs/tcu2rcu"
       :com (roslisp:make-msg 
             "dlr_msgs/tcu2rcu_Com"
             :command (get-beasty-command-code 
                       (infer-command-symbol parameters))
             :cmd_id (cmd-id interface)
             :session_id (session-id interface))
       :robot robot-msg
       :controller controller-msg
       :interpolator interpolator-msg
       :settings settings-msg))))