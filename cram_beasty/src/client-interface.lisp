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
;;; CL-TRANSFORMS
;;;

(defun transpose-2d-matrix (matrix)
  (declare (type array matrix))
  (assert (= (array-rank matrix) 2))
  (destructuring-bind (rows columns) (array-dimensions matrix)
    (make-array 
     `(,columns ,rows)
     :initial-contents
     (loop for column from 0 below columns collecting
       (loop for row from 0 below rows collecting
         (aref matrix row column))))))

;;;
;;; ACTUAL INTERFACE
;;;

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
     client (make-beasty-goal-msg goal-description) 
     exec-timeout preempt-timeout)
    (if (action-succeeded-p client)
        (make-instance
         'beasty-handle
         :client client
         :session-id (msg-slot-value (result client) :session_id)
         :cmd-id (extract-cmd-id (result client) command-code))
        (error "Login to Beasty '~S' with ID '~S' and PWD '~S' failed."
               action-name user-id user-pwd))))

(defun move-beasty-and-wait (handle goal-description
                             &key (exec-timeout *exec-timeout*) 
                               (preempt-timeout *preemt-timeout*))
  (flet ((add-com-description (goal-description handle)
           (with-slots (session-id cmd-id) handle
             (setf (getf goal-description :cmd-id) cmd-id)
             (setf (getf goal-description :session-id) session-id))
           goal-description))
  (with-recursive-lock ((lock handle))
    (send-goal-and-wait
     (client handle) 
     (goal-description-to-msg (add-com-description goal-description handle))
     exec-timeout 
     preempt-timeout)
    (update-cmd-id handle goal-description))))