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

(in-package :cram-fccl)

(defclass fccl-action-interface ()
  ((action-client :initarg :action-client :reader action-client)
   (kinematic-chain :initarg :kinematic-chain :reader kinematic-chain)))

(defun make-fccl-action-interface (action-client kinematic-chain)
  (declare (type kinematic-chain kinematic-chain)
           (type actionlib::action-client action-client))
  (actionlib:wait-for-server action-client 2.0)
  (make-instance 'fccl-action-interface
                 :action-client action-client
                 :kinematic-chain kinematic-chain))

(defgeneric execute-fccl-motion (interface motion cancel-callback &key execution-timeout))

(defmethod execute-fccl-motion ((interface fccl-action-interface) (motion list)
                                (cancel-callback function) &key execution-timeout)
  (handler-bind ((actionlib:feedback-signal 
                   (lambda (feedback-signal)
                     (with-slots ((goal-handle actionlib::goal) 
                                  (feedback actionlib::feedback)) 
                         feedback-signal
                       (declare (ignore goal-handle))
                       (when (funcall cancel-callback feedback)
                         (invoke-restart 'actionlib:abort-goal))))))
    (actionlib:send-goal-and-wait (action-client interface)
                                  (actionlib:make-action-goal
                                      (action-client interface)
                                    :constraints (to-msg motion)
                                    :kinematics (to-msg (kinematic-chain interface)))
                                  :exec-timeout execution-timeout)))