;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
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

(in-package :cram-wsg50)

(defclass wsg50-handle ()
  ((namespace :initarg :namespace :reader namespace
              :documentation "Namespace for the topics.")
   (goal-position-pub :initarg :pub :accessor goal-position-pub
                      :documentation "Publisher for the goal_position topic.")
   (status-sub :initarg :sub :reader status-sub
               :documentation "Subscriber for the status topic of the gripper.")
   (status-fluent :reader status-fluent
                  :documentation "Fluent holding the status of the gripper."))
  (:documentation "Handle for sending commands to the wsg50 gripper."))

(defmethod status-fluent :before ((handle wsg50-handle))
  "If the status-fluent slot is unbound it subscribes to the status topic and creates a fluent with the values of the status message."
  (unless (slot-boundp handle 'status-fluent)
    (let* ((fluent (cpl-impl:make-fluent))
           (sub (subscribe (concatenate 'string (namespace handle) "/status")
                           "iai_wsg_50_msgs/Status"
                           (lambda (msg)
                             (setf (cpl-impl:value fluent) 
                                   (read-status msg))))))
      (setf (slot-value handle 'status-sub) sub)
      (setf (slot-value handle 'status-fluent) fluent))))

(defun read-status (msg)
  "Returns a plist with the names and values of the status msg."
  `(:status ,(iai_wsg_50_msgs-msg:status msg)
    :width ,(iai_wsg_50_msgs-msg:width msg)
    :speed ,(iai_wsg_50_msgs-msg:speed msg)
    :acc ,(iai_wsg_50_msgs-msg:acc msg)
    :force ,(iai_wsg_50_msgs-msg:force msg)
    :force_finger0 ,(iai_wsg_50_msgs-msg:force_finger0 msg)
    :force_finger1 ,(iai_wsg_50_msgs-msg:force_finger1 msg)))

(defun make-wsg50-handle (namespace)
  "Creates and returns a wsg50-handle."
  (let ((pub (advertise (concatenate 'string namespace "/goal_position")
                        "iai_wsg_50_msgs/PositionCmd")))
    (make-instance 'wsg50-handle :namespace namespace :pub pub)))

(defun move-wsg50 (handle pos speed force)
  "Moves the gripper to the given position in mm with speed in mm/s and force in N."
  (publish-msg (goal-position-pub handle) :pos pos :speed speed :force force))