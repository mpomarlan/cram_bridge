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

(in-package :cram-beliefstate)

(defun init-interactive-control ()
  (setf *interactive-callback-subscriber*
        (roslisp:subscribe "/interactive_callback"
                           "designator_integration_msgs/Designator"
                           #'interactive-callback)))

(defun interactive-callback (msg)
  (let* ((desig (setf (cpl:value *interactive-callback-fluent*)
                      (designator-integration-lisp::msg->designator
                       msg)))
         (callback (cdr (find (desig-prop-value desig 'desig-props::command)
                              *registered-interactive-callbacks*
                              :test (lambda (x y)
                                      (string= x (car y)))))))
    (when callback
      (funcall callback
               (desig-prop-value desig 'desig-props::command)
               (desig-prop-value desig 'desig-props::object)
               (desig-prop-value desig 'desig-props::parameter)))))

(roslisp-utilities:register-ros-init-function init-interactive-control)

(defun register-interactive-object (name shape pose dimensions menu)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'register-interactive-object)
          (list 'name name)
          (list 'shape shape)
          (list 'pose pose)
          (list 'width (elt dimensions 0))
          (list 'depth (elt dimensions 1))
          (list 'height (elt dimensions 2))
          (list 'menu menu)))))

(defun unregister-interactive-object (name)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'unregister-interactive-object)
          (list 'name name)))))

(defun set-interactive-object-menu (name menu)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'set-interactive-object-menu)
          (list 'name name)
          (list 'menu menu)))))

(defun set-interactive-object-pose (name pose)
  (alter-node
   (cram-designators:make-designator
    'cram-designators:action
    (list (list 'command 'set-interactive-object-pose)
          (list 'name name)
          (list 'pose pose)))))

(defun register-interactive-callback (command callback-function)
  "Callback functions need to receive two parameters: one for the
object name clicked, and one for the optional parameter given when the
menu entry for the interactive object was assigned."
  (setf *registered-interactive-callbacks*
        (remove command *registered-interactive-callbacks*
                :test (lambda (x y)
                        (string= x (car y)))))
  (push (cons command callback-function)
        *registered-interactive-callbacks*))

(defun unregister-interactive-callback (command callback-function)
  (setf *registered-interactive-callbacks*
        (remove command *registered-interactive-callbacks*
                :test (lambda (x y)
                        (and (string= x (car y))
                             (eql callback-function (cdr y)))))))

(defmacro with-interactive-objects (defs &body body)
  `(let ((defined-functions nil))
     ,@(mapcar (lambda (def)
                 (destructuring-bind (command-name) def
                   `(let ((def-func (lambda (command object parameter)
                                      (format t "Received: ~a ~a ~a"
                                              command object parameter))))
                      (register-interactive-object
                       ',command-name def-func)
                      (push (cons ',command-name def-func) defined-functions))))
               defs)
     (unwind-protect
          ,@body
       (dolist (pair defined-functions)
         (destructuring-bind (command-name . defined-function) pair
           (unregister-interactive-object command-name defined-function))))))
