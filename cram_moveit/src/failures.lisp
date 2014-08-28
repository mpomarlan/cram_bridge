;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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

;; Generic failure conditions
(define-condition moveit-failure (manipulation-failure) ())

;; MoveIt! specific failure conditions
(define-condition planning-failed (moveit-failure) ())
(define-condition invalid-motion-plan (moveit-failure) ())
(define-condition motion-plan-invalidated-by-environment-change
    (moveit-failure) ())
(define-condition control-failed (moveit-failure) ())
(define-condition unable-to-acquire-sensor-data (moveit-failure) ())
(define-condition timed-out (moveit-failure) ())
(define-condition preempted (moveit-failure) ())
(define-condition start-state-in-collision (moveit-failure) ())
(define-condition start-state-violates-path-constraints (moveit-failure) ())
(define-condition goal-in-collision (moveit-failure) ())
(define-condition goal-violates-path-constraints (moveit-failure) ())
(define-condition goal-constraints-violated (moveit-failure) ())
(define-condition invalid-group-name (moveit-failure) ())
(define-condition invalid-goal-constraints (moveit-failure) ())
(define-condition invalid-robot-state (moveit-failure) ())
(define-condition invalid-link-name (moveit-failure) ())
(define-condition invalid-object-name (moveit-failure) ())
(define-condition frame-transform-failure (moveit-failure) ())
(define-condition collision-checking-unavailable (moveit-failure) ())
(define-condition robot-state-stale (moveit-failure) ())
(define-condition sensor-info-stale (moveit-failure) ())
(define-condition no-ik-solution (moveit-failure) ())

;; Specialized failure-conditions
(define-condition no-collision-shapes-defined () ())
(define-condition pose-not-transformable-into-link () ())

(defvar *known-failures* nil
  "List of known failures and their respective error code as returned
  by MoveIt!.")

(defun register-known-moveit-errors ()
  "Registers all known MoveIt! failure conditions with their
  respective error codes as returned by MoveIt!."
  (register-moveit-error 99999 'moveit-failure)
  (register-moveit-error -1 'planning-failed)
  (register-moveit-error -2 'invalid-motion-plan)
  (register-moveit-error -3 'motion-plan-invalidated-by-environment-change)
  (register-moveit-error -4 'control-failed)
  (register-moveit-error -5 'unable-to-acquire-sensor-data)
  (register-moveit-error -6 'timed-out)
  (register-moveit-error -7 'preempted)
  (register-moveit-error -10 'start-state-in-collision)
  (register-moveit-error -11 'start-state-violates-path-constraints)
  (register-moveit-error -12 'goal-in-collision)
  (register-moveit-error -13 'goal-violates-path-constraints)
  (register-moveit-error -14 'goal-constraints-violated)
  (register-moveit-error -15 'invalid-group-name)
  (register-moveit-error -16 'invalid-goal-constraints)
  (register-moveit-error -17 'invalid-robot-state)
  (register-moveit-error -18 'invalid-link-name)
  (register-moveit-error -19 'invalid-object-name)
  (register-moveit-error -21 'frame-transform-failure)
  (register-moveit-error -22 'collision-checking-unavailable)
  (register-moveit-error -23 'robot-state-stale)
  (register-moveit-error -24 'sensor-info-stale)
  (register-moveit-error -31 'no-ik-solution))

(defun register-moveit-error (error-code condition)
  "Registers a new condition with an error code so that the respective
  error condition can be triggered, based on a given error code
  value."
  (unless (assoc error-code *known-failures*)
    (push (cons error-code condition) *known-failures*)))

(defun signal-moveit-error (error-code &rest arguments)
  "Signals the error condition identified by the given error code
  `error-code'. Signals a generic `moveit-failure' condition when the
  error-code could not be identified. If the error code coincides with
  the SUCCESS constant in the MoveIt! message definition, no error is
  thrown."
  (unless
      (eql error-code
           (roslisp-msg-protocol:symbol-code
            'moveit_msgs-msg:moveiterrorcodes
            :success))
    (let ((condition (cdr (assoc error-code *known-failures*))))
      (ros-error (moveit) "Signalling: ~a" condition)
      (cond (condition
             (error condition arguments))
            (t (error 'moveit-failure arguments))))))
