;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-uima)

(defparameter *uima-action-client* nil)

(defparameter *uima-action-name* "/RoboSherlock_common/some_action")
(defparameter *init-timeout* 3.0)
(defparameter *exec-timeout* 5.0)
(defparameter *preempt-timeout* 3.0)

(defun init-uima-action-bridge (&key (action-name *uima-action-name*) (timeout *init-timeout*))
  (init-uima-action-client action-name timeout)
  (init-uima-result-fluent))

(defun destroy-uima-action-bridge ()
  (destroy-uima-action-client)
  (destroy-uima-result-fluent))

(defun query-uima (&key (desigs nil) (exec-timeout *exec-timeout*) 
                     (preempt-timeout *preempt-timeout*))
  "Queries UIMA for the objects it currently perceives. `desigs' denots a list
 of all the object designators we are looking for."
  (when *uima-action-client*
    (actionlib-lisp:send-goal-and-wait 
     *uima-action-client* 
     (make-uima-action-goal desigs)
     exec-timeout preempt-timeout)
    (when (eql (actionlib-lisp:state *uima-action-client*) :SUCCEEDED)
      (unpack-uima-action-result (actionlib-lisp:result *uima-action-client*)))))
;;;
;;; INTERNAL
;;;

(defun init-uima-action-client (action-name timeout)
  (unless *uima-action-client*
    (setf *uima-action-client*
          (actionlib-lisp:make-simple-action-client 
           action-name "iai_robosherlock_actions/SimplePerceiveObjectAction"))
    (actionlib-lisp:wait-for-server *uima-action-client* timeout)))

(defun init-uima-result-fluent ()
  (unless *uima-result-fluent*
    (setf *uima-result-fluent*
          (cpl:make-fluent :name 'uima-result 
                           :value nil 
                           :allow-tracing nil))))

(defun destroy-uima-action-client ()
  (when *uima-action-client*
    (actionlib-lisp:stop-tracking-goal *uima-action-client*)
    (setf *uima-action-client* nil)))

(defun destroy-uima-result-fluent ()
  (when *uima-result-fluent*
    (setf *uima-result-fluent* nil)))

(defun make-uima-action-goal (desigs)
  "Creates an action goal for UIMA from `desigs', a list of designators."
  (roslisp:make-message
   "iai_robosherlock_actions/SimplePerceiveObjectGoal"
   :requests (coerce (mapcar #'desig-int::designator->msg desigs) 'vector)))

(defun unpack-uima-action-result (result-msg)
  "Returns all perceived objects designators returned from UIMA in `result-msg'."
  (roslisp:with-fields (percepts) result-msg
    (mapcar #'desig-int::msg->designator (coerce percepts 'list))))