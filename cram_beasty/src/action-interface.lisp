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

;;; EXPORTED INTERFACE METHODS

(defun make-beasty-interface (action-name robot &optional (visualization-on t))
  "Creates a BEASTY interface. `action-name' is the name of the action used to create
   the ROS action-client and will also be used to identify the BEASTY interface. `robot'
   is expected to be an instance of 'beasty-robot' modelling the setup of the LWR arm.
   `visualization-on' is a boolean flag to trigger publishing of visualization markers."
  (declare (type string action-name)
           (type beasty-robot robot)
           (type boolean visualization-on))
  (let ((action-client (actionlib:make-action-client action-name "dlr_msgs/RCUAction"))
        ;; TODO(Georg): get this out of here!
        (visualization-pub 
          (when visualization-on
            (advertise "visualization_marker_array" "visualization_msgs/MarkerArray"))))
    (actionlib:wait-for-server action-client 2.0)
    (multiple-value-bind (session-id cmd-id)
        (login-beasty action-client)
      (let ((interface (make-instance 'beasty-interface 
                                      :action-client action-client
                                      :visualization-pub visualization-pub
                                      :session-id session-id
                                      :cmd-id cmd-id
                                      :robot robot)))
        (add-state-subscriber interface action-name)
        interface))))

(defun cleanup-beasty-interface (interface)
  "Kicks in the brakes of the LWR behind `interface' and logouts out of beasty. NOTE: This
 is brutal on the arm when called during motion!"
  (declare (type beasty-interface))
  (command-beasty interface (make-instance 'hard-stop-parameters))
  (logout-beasty interface))

(defun command-beasty (interface parameters 
                       &optional (safety (make-instance 'safety-settings)))
  "Sends a command to beasty controller behind `interface'. Users can alter motion command
 with `parameters' and `safety'."
  (declare (type beasty-interface interface)
           (type safety-settings safety))
  (reset-safety interface safety)
  (let* ((command-code (get-beasty-command-code (infer-command-symbol parameters)))
         (goal (actionlib:make-action-goal (action-client interface)
                 :command command-code
                 :parameters (make-parameter-msg interface parameters safety))))
    (send-cancelable-goal-to-beasty interface goal command-code)))

(defun cancel-command (interface)
  "Cancels current goal executed by `interface'. Is ignored if there is no current goal."
  (declare (type beasty-interface interface))
  (setf (cancel-request interface) t))

;;; SOME INTERNAL AUXILIARY METHODS

(defun send-cancelable-goal-to-beasty (interface goal-msg command-code)
  "Sends `goal-msg' to server behind `interface-msg'. `command-code' is command code used
 when creating `goal-msg'. Goal can be cancel at any time by calling 'cancel-goal'."
  (declare (type beasty-interface interface)
           (type dlr_msgs-msg:rcugoal goal-msg)
           (type number command-code))
  (setf (cancel-request interface) nil)
  (handler-bind ((actionlib:feedback-signal 
                   (lambda (fb-signal) 
                     (declare (ignore fb-signal))
                     (when (cancel-request interface)
                       (invoke-restart 'actionlib:abort-goal)
                       (setf (cancel-request interface) nil)))))
    (send-non-cancelable-goal-to-beasty interface goal-msg command-code)))

(defun send-non-cancelable-goal-to-beasty (interface goal-msg command-code)
  "Sends `goal-msg' to server behind `interface-msg'. `command-code' is command code used
 when creating `goal-msg'. Goal cannot be cancel from outside."
  (declare (type beasty-interface interface)
           (type dlr_msgs-msg:rcugoal goal-msg)
           (type number command-code))
  (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait (action-client interface) goal-msg)
      (unless (equal :succeeded status)
        (error 'beasty-command-error :test "Error commanding beasty action interface."))
    (update-cmd-id interface result command-code)))

(defun update-cmd-id (interface result-msg command-code)
  "Updates the command-id kept in `interface' as reported from beasty in `result-msg'.
 `command-code' is command code used when creating the original goal-msg."
  (declare (type beasty-interface interface)
           (type dlr_msgs-msg:rcuresult result-msg)
           (type number command-code))
  (with-fields (state) result-msg
    (with-fields (com) state
      (with-fields (cmd_id) com
        (setf (cmd-id interface) (elt cmd_id command-code))))))
  
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
                                     (from-msg msg))
                               ;; TODO(Georg): Visualization should be triggered by change
                               ;;              to state-fluent. Move this away from here..
                               (visualize-collisions interface))
                             :max-queue-length 1)))
    (setf (state-sub interface) subscriber)))

(defun infer-command-symbol (parameters)
  "Infers the RCUGoal-command type based on the type of `parameters'."
  (etypecase parameters
    (gravity-control-parameters :CHANGE_BEHAVIOUR)
    (joint-impedance-control-parameters :MOVETO)
    (cartesian-impedance-control-parameters :MOVETO)
    (hard-stop-parameters :STOP)
    (safety-reset :RESET_SAFETY)))
                           
(defun make-parameter-msg (interface parameters safety)
  "Creates the appropriate parameter message to control `robot' behind `interface' to
   perform motion specified by `parameters' with `safety'."
  (declare (type beasty-interface interface)
           (type safety-settings safety))
  (ensure-correct-emergency-status (robot interface) parameters)
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
       :settings settings-msg
       :safety (to-msg safety)))))

(defun ensure-correct-emergency-status (robot parameters)
  "Sets the 'emergency-released-flag' of `robot' to 'nil' if `parameters' is of type
 'hard-stop-paramterers', else sets it to 't'."
  (declare (type beasty-robot robot))
  (setf (emergency-released-flag robot) (not (typep parameters 'hard-stop-parameters))))  

(defun reset-safety (interface safety)
  (declare (type beasty-interface interface)
           (type safety-settings interface))
  (let* ((params (make-instance 'safety-reset))
         ;; TODO(Georg): refactor this into a method which returns command-code and goal
         (command-code (get-beasty-command-code (infer-command-symbol params)))
         (goal (actionlib:make-action-goal (action-client interface)
                 :command command-code
                 :parameters (make-parameter-msg interface params safety))))
    (send-non-cancelable-goal-to-beasty interface goal command-code)))