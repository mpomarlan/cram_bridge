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

;;; SOME INTERNAL AUXILIARY METHODS

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

;;; PUBLISHING OF VISUALIZATION MARKERS
;;; TODO(Georg): move this into a separate file

(defun visualize-collisions (interface)
  "Visualizes collisions reported in feedback of beasty `interface' by publishing a red
 sperical marker at every joint which is in collision."
  (declare (type beasty-interface interface))
  ;; Note: If the user specified 'no-visualization' the publisher object is 'nil'.
  (when (visualization-pub interface)
    (let ((collision-markers 
            (create-collision-markers
             (joint-collisions (cram-language:value (state interface))))))
      ;; only publish if there is something to publish
      (when (> (length collision-markers) 0)
        (publish (visualization-pub interface)
                 (make-msg "visualization_msgs/MarkerArray" :markers collision-markers))))))
  
(defun create-collision-markers (collision-joints)
  "Returns vector of collision markers for joints which are in collision. 
 `joint-collisions' is a vector expected to be joint collision feedback comming from
 Beasty, while `joint-prefix' is a string."
  (declare (type vector collision-joints))
  (map 'vector #'make-red-sphere-msg collision-joints))

;; TODO(Georg): extend with namespace to discriminate btw. arms
(defun make-red-sphere-msg (joint-name)
  "Creates a red sphere marker for joint with `joint-name' located at the corresponding
 link of the kinematic chain. Joint names are expect to match *_<joint-number>_joint."
  (declare (type string joint-name))
  (let ((frame-id (get-joint-frame joint-name))
        (joint-index (get-joint-index joint-name)))
    (make-msg "visualization_msgs/Marker"
              :header (make-msg "std_msgs/Header" :frame_id frame-id :stamp (ros-time))
              :type 2 :action 0 :lifetime 5.0 :id joint-index
              :color (make-msg "std_msgs/ColorRGBA" :r 1.0 :g 0.0 :b 0.0 :a 0.7)
              :scale (make-msg "geometry_msgs/Vector3" :x 0.2 :y 0.2 :z 0.2))))

(defun get-joint-index (joint-name)
  "Parses `joint-name' which is expected to match pattern *<index>* for the index of the
 joint and returns it as number."
  (declare (type string joint-name))
  (multiple-value-bind (joint-index string-position)
      (parse-integer (remove-if-not #'digit-char-p joint-name) :junk-allowed t)
    (declare (ignore string-position))
    joint-index))

(defun get-joint-frame (joint-name)
  "Returns the corresponding tf link-name for joint with `joint-name' of pattern
 *<index>*_joint by replacing 'joint' with 'link', and incrementing 'index'."
  (declare (type string joint-name))
  (let ((joint-index (get-joint-index joint-name)))
    (when joint-index
      (let* ((joint-index-position (search (write-to-string joint-index) joint-name))
             (prefix (subseq joint-name 0 joint-index-position))
             (postfix (subseq joint-name (incf joint-index-position) (length joint-name)))
             (inc-joint-name
               (concatenate 'string prefix (write-to-string (incf joint-index)) postfix)))
        (concatenate 'string (subseq inc-joint-name 0 
                                     (search "joint" inc-joint-name)) "link")))))