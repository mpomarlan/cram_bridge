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

(in-package :cram-beasty)

;;;
;;; TODO(Georg): Move this to data-structures?
;;;

(defclass beasty-handle ()
  ((client :initarg :client :reader client :type actionlib-lisp::action-client
           :documentation "For internal use. ROS action client to talk to Beasty.")
   (lock :initform (make-mutex :name (string (gensym "BEASTY-LOCK-")))
         :accessor lock :type mutex
         :documentation "For internal use. Mutex to guard Beasty-handle.")
   (session-id :initarg :session-id :accessor session-id :type number
               :documentation "For internal use. ID of current communication session.")
   (cmd-id :initarg :cmd-id :accessor cmd-id :type number
           :documentation "For internal use. cmd-id to be used in the next goal."))
  (:documentation "Handle to talk with a Beasty controller for the LWR."))

(defun make-beasty-handle (action-name user-id user-pwd
                           &key (exec-timeout *exec-timeout*) 
                             (preempt-timeout *preemt-timeout*)
                             (server-timeout *server-timeout*))
  (let* ((client (make-simple-action-client
                  action-name *beasty-action-type*))
         (command-code (get-beasty-command-code :LOGIN))
         (goal-description 
           `(:command ,command-code
                      (:command :com :parameters) ,command-code
                      (:user_id :com :parameters) ,user-id
                      (:user_pwd :com :parameters) ,user-pwd)))
    (wait-for-server client server-timeout)
    (send-goal-and-wait 
     client (make-beasty-goal goal-description) 
     exec-timeout preempt-timeout)
    (if (action-succeeded-p client)
        (make-instance
         'beasty-handle
         :client client
         :session-id (msg-slot-value (result client) :session_id)
         :cmd-id (extract-cmd-id (result client) command-code))
        (error "Login to Beasty '~S' with ID '~S' and PWD '~S' failed."
               action-name user-id user-pwd))))