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

(defparameter *uima-action-name* "/RoboSherlock_common/some_action")
(defparameter *uima-action-type* "iai_robosherlock_actions/SimplePerceiveObjectAction")
(defparameter *init-timeout* 3.0)
(defparameter *exec-timeout* 5.0)
(defparameter *preempt-timeout* 3.0)

(defclass uima-action-interface ()
  ((action-client :initarg :action-client :reader action-client
                  :type actionlib-lisp:simple-action-client))
  (:documentation "Action interface to UIMA perception system."))

(defun make-uima-action-interface (&key (action-name *uima-action-name*)
                                       (action-type *uima-action-type*)
                                       (timeout *init-timeout*))
  "Creates an UIMA action interface with `action-name', and `action-type'.
 Waits for the server show for `timeout' seconds."
  (let ((action-client (actionlib-lisp:make-simple-action-client 
                        action-name action-type)))
    (actionlib-lisp:wait-for-server action-client timeout)
    (make-instance 'uima-action-interface :action-client action-client)))

(defun cleanup-uima-action-interface (interface)
  "Cleans up UIMA `interface', i.e. tears down action-client. Returns the cleaned
 `interface'"
  (declare (type uima-action-interface interface))
  (with-slots (action-client) interface
    (actionlib-lisp:stop-tracking-goal action-client)
    (setf action-client nil)
    interface))
    
(defun query-uima-and-wait (interface &key (desigs nil) (exec-timeout *exec-timeout*) 
                                        (preempt-timeout *preempt-timeout*))
  "Queries UIMA `interface' for the objects it currently perceives, and waits for the
 result. Perceived objects will be returned as a list of designators. `desigs' denots a
 list of all the object designators we are looking for."
  (declare (type uima-action-interface interface))
  (with-slots (action-client) interface
    (send-uima-goal-and-wait action-client desigs exec-timeout preempt-timeout)
    (get-successful-uima-results action-client)))

;;;
;;; INTERNAL
;;;

(defun send-uima-goal-and-wait (action-client desigs exec-timeout preempt-timeout)
  (actionlib-lisp:send-goal-and-wait 
   action-client
   (actionlib-lisp:make-action-goal-msg action-client
     :requests (coerce (mapcar #'desig-int::designator->msg desigs) 'vector))
   exec-timeout preempt-timeout))

(defun get-successful-uima-results (action-client)
  (when (eql (actionlib-lisp:state action-client) :SUCCEEDED)
    (roslisp:with-fields (percepts) (actionlib-lisp:result action-client)
      (mapcar #'desig-int::msg->designator (coerce percepts 'list)))))