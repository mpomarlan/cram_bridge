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

(in-package :cram-fccl)

(defclass fccl-action-client ()
  ((action-client :initarg :action-client :reader action-client
                  :type actionlib::action-client)
   (kinematic-chain :initarg :kinematic-chain :reader kinematic-chain
                    :type kinematic-chain)
   (cancel-request :initform nil :accessor cancel-request :type boolean)
   (execution-lock :initform (make-mutex :name (string (gensym "FCCL-EXECUTION-LOCK-")))
                   :accessor execution-lock :type mutex)
   (data-lock :initform (make-mutex :name (string (gensym "FCCL-DATA-LOCK-")))
              :accessor data-lock :type mutex)))

(defmethod make-fccl-action-client ((action-client actionlib::action-client)
                                (kinematics kinematic-chain))
  (actionlib:wait-for-server action-client 2.0)
  (make-instance 
   'fccl-action-interface :action-client action-client :kinematic-chain kinematics))

(defmethod make-fccl-action-client ((action-name string) (kinematics kinematic-chain))
  (make-fccl-action-client
   (actionlib:make-action-client action-name "fccl_msgs/SingleArmMotionAction")
   kinematics))

(defmethod command-motion ((interface fccl-action-client) (motion motion-phase))
  (with-recursive-lock ((execution-lock interface))
    (send-cancelable-goal interface (make-single-arm-motion-goal interface motion))))

(defun make-single-arm-motion-goal (interface motion)
  (declare (type fccl-action-client)
           (type motion-phase motion))
  (actionlib:make-action-goal (action-client interface)
    :constraints (to-msg motion)
    :kinematics (to-msg (kinematic-chain interface))))

(defun send-cancelable-goal (interface goal-msg)
  (declare (type fccl-action-client interface)
           (type fccl_msgs-msg:singlearmmotiongoal goal-msg))
  (with-recursive-lock ((execution-lock interface))
    (set-cancel-request-signal interface nil)
    (handler-bind ((actionlib:feedback-signal 
                     (lambda (fb-signal) 
                       (declare (ignore fb-signal))
                       (when (cancel-requested-p interface)
                         (invoke-restart 'actionlib:abort-goal)
                         (set-cancel-request-signal interface nil)))))
      (send-non-cancelable-goal interface goal-msg))))

(defun send-non-cancelable-goal (interface goal-msg)
  (declare (type fccl-action-client interface)
           (type fccl_msgs-msg:singlearmmotiongoal goal-msg))
  (with-recursive-lock ((execution-lock interface))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait (action-client interface) goal-msg)
      (declare (ignore status)) ; TODO(Georg): think about a good low-level interface
      result)))

(defmethod cancel-motion ((interface fccl-action-client))
  (set-cancel-request-signal interface t)
  (with-recursive-lock ((execution-lock interface)) nil))

(defun set-cancel-request-signal (interface cancel-requested-p)
  (declare (type fccl-action-client interface)
           (type boolean cancel-requested-p))
  (with-recursive-lock ((data-lock interface))
    (setf (cancel-request interface) cancel-requested-p)))

(defun cancel-requested-p (interface)
  (declare (type fccl-action-client interface))
  (with-recursive-lock ((data-lock interface)) (cancel-request interface)))