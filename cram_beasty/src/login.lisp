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

(defparameter *beasty-user-id* 1001
  "User ID provided by Sven.")

(defparameter *beasty-user-pwd* 1234
  "User Password provided by Sven.")

(define-condition beasty-login-error (error)
  ((test :initarg :text :reader text)))

(defun login-beasty (action-client &optional (user-id *beasty-user-id*)
                                     (user-pwd *beasty-user-pwd*))
  "Tries to login into BEASTY controller behind `action-client' using the specified
   `user-id' and `user-pwd'. If not successful will signal an 'beasty-login-error'.
   Otherwise returns the 'session-id' and 'cmd-id' returned from the controller."
  (declare (type actionlib::action-client action-client)
           (type number user-id user-pwd))
  (multiple-value-bind (result status)
      (actionlib:send-goal-and-wait action-client 
                                    (make-login-goal action-client user-id user-pwd))
    (unless (equal :succeeded status)
      (error 'beasty-login-error :text "Login into BEASTY controller failed."))
    (with-fields (state) result
      (with-fields (com) state
        (with-fields (session_id cmd_id) com
          (values session_id (elt cmd_id 9)))))))
    
(defun make-login-goal (action-client user-id user-pwd)
  "Generates an action-goal to log into Beasty controller serving `action-client' using
   `user-id' and `user-pwd'."
  (declare (type actionlib::action-client action-client)
           (type number user-id user-pwd))
  (actionlib:make-action-goal action-client
    :command (get-beasty-command-code :LOGIN)
    :parameters (roslisp:make-msg 
                 "dlr_msgs/tcu2rcu"
                 :com (roslisp:make-msg 
                       "dlr_msgs/tcu2rcu_Com"
                       :user_id user-id
                       :user_pwd user-pwd
                       :command (get-beasty-command-code :LOGIN)))))