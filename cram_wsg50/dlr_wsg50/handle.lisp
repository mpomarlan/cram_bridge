;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-dlr-wsg50)

(defun make-dlr-wsg50-handle (action-name &optional (timeout (default-login-timeout)))
  (let ((client
          (actionlib-lisp:make-simple-action-client action-name "dlr_msgs/GripperAction")))
    (actionlib-lisp:wait-for-server client timeout)
    client))

(defun cmd-wsg50-and-wait (handle command width &key (speed (default-speed))
                         (acceleration (default-acceleration))
                         (force (default-force))
                         (limit-min (default-limit-min))
                         (limit-max (default-limit-max))
                         (exec-timeout (default-exec-timeout))
                         (preempt-timeout (default-preempt-timeout)))
  ;;; Rico said that their firmware sometime gets into a state where new
  ;;; commands need to be prepended with a STOP and an ACKNOWLEDGE. As I
  ;;; did not understand how to detect that state, I am just unconditionally
  ;;; sending these commands before any actual command. They should not interfere
  ;;; with anything but make the gripper a bit slower.
  (stop-wsg50-and-wait handle)
  (acknowledge-wsg50-and-wait handle)
  (cmd-wsg50-and-wait-internal
   handle command width speed acceleration force
   limit-min limit-max exec-timeout preempt-timeout))

(defun acknowledge-wsg50-and-wait (handle &key (exec-timeout (default-acknowledge-timeout))
                                          (preempt-timeout (default-preempt-timeout)))
  (cmd-wsg50-and-wait-internal
   handle :acknowledge 0 (default-speed) (default-acceleration) (default-force)
   (default-limit-min) (default-limit-max) exec-timeout preempt-timeout))

(defun stop-wsg50-and-wait (handle &key (exec-timeout (default-stop-timeout))
                                          (preempt-timeout (default-preempt-timeout)))
  (cmd-wsg50-and-wait-internal
   handle :stop 0 (default-speed) (default-acceleration) (default-force)
   (default-limit-min) (default-limit-max) exec-timeout preempt-timeout))
                               

(defun default-speed ()
  0.2)

(defun default-force ()
  80)

(defun default-acceleration ()
  1)

(defun default-limit-min ()
  0.005)

(defun default-limit-max ()
  0.1)

(defun default-exec-timeout ()
  3)

(defun default-preempt-timeout ()
  0.5)

(defun default-login-timeout ()
  2)

(defun default-acknowledge-timeout ()
  0.5)

(defun default-stop-timeout ()
  0.5)

;;;
;;; INTERNAL AUX
;;;

(defun cmd-wsg50-and-wait-internal (handle command width speed acceleration
                                    force limit-min limit-max exec-timeout preempt-timeout)
  (let* ((goal (make-goal handle command width speed acceleration force limit-min limit-max)))
    (actionlib-lisp:send-goal-and-wait handle goal exec-timeout preempt-timeout)))

(defun make-goal (handle command width speed acceleration 
                  force limit-min limit-max)
  (declare (type keyword command))
  (actionlib-lisp:make-action-goal-msg handle
    :command (keyword->symbol-code command)
    :width width
    :speed speed
    :acceleration acceleration
    :force force
    :limit_min limit-min
    :limit_max limit-max))

(defun goal-symbol-code (code)
  (symbol-code 'dlr_msgs-msg:grippergoal code))

(defun keyword->symbol-code (keyword)
  (goal-symbol-code
   (ecase keyword
     (:grasp :cmd_grasp_part)
     (:release :cmd_release_part)
     (:move :cmd_move)
     (:stop :cmd_stop)
     (:home :cmd_homing)
     (:acknowledge :cmd_ack))))
