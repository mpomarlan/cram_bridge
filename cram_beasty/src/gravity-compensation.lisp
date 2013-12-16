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
  ((text :initarg :text :reader text)))

(defun command-gravity (interface robot parameters safety)
  (declare (type beasty-robot robot)
           (type beasty-control-parameters))
  
  (let* ((command :CHANGE_BEHAVIOR)
         (result (command-beasty-action command interface robot parameters safety)))
    (with-fields (state) result
      (with-fields (com) state
        (with-fields (cmd_id) com
          (setf (cmd-id interface) (elt cmd_id (get-beasty-command-code command))))))))

(defun make-gravity-goal (interface robot parameters safety)
  (declare (type beasty-interface interface)
           (type beasty-robot robot)
           (type beasty-control-parameters parameters)
           (ignore safety))
  (multiple-value-bind (robot-msg settings-msg) (to-msg robot)
    (actionlib:make-action-goal (action-client interface)
      :command (get-beasty-command-code :CHANGE_BEHAVIOUR)
      :parameters (roslisp:make-msg "dlr_msgs/tcu2rcu"
                                    :com (roslisp:make-msg 
                                          "dlr_msgs/tcu2rcu_Com"
                                          :command (get-beasty-command-code 
                                                    :CHANGE_BEHAVIOUR)
                                          :cmd_id (cmd-id interface)
                                          :session_id (session-id interface))
                                    :robot robot-msg
                                    :controller (roslisp:make-msg
                                                 "dlr_msgs/tcu2rcu_Controller"
                                                 :mode 3)
                                    :interpolator (to-msg parameters)
                                    :settings settings-msg))))