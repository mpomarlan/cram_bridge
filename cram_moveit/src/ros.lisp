;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
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

(in-package :cram-moveit)

(defvar *move-group-action-client* nil)
(defvar *moveit-action-access-lock* nil)

(defun connect-action-client ()
  (setf *move-group-action-client*
        (actionlib-lisp::make-action-client
         "/move_group" "moveit_msgs/MoveGroupAction"
         20.0)))

(defmethod send-goal (client goal &key (timeout 0.0))
  (with-lock-held (*moveit-action-access-lock*)
    (actionlib-lisp:wait-for-server client)
    (let* ((wait-fluent (cpl:make-fluent))
           (goal-handle
             (actionlib-lisp:send-goal
              client goal
              :transition-cb (lambda (goal-handle)
                               (let ((status (actionlib-lisp:comm-state goal-handle)))
                                 (when (eql status :done)
                                   (cpl-impl:pulse wait-fluent)))))))
      (cpl:wait-for (cpl-impl:fl-pulsed wait-fluent))
      (let ((result (actionlib-lisp:result goal-handle)))
        (unless result
          (ros-error (moveit) "Empty actionlib response.")
          (error 'actionlib:server-lost))
        result))))

(defmacro send-action (client &rest args)
  `(let ((goal (actionlib-lisp:make-action-goal-msg ,client ,@args)))
     (send-goal ,client goal)))
