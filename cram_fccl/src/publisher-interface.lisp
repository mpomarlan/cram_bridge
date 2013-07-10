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

(in-package :cram-fccl)

(defvar *fccl-controllers* nil "Hashtable from controller-name to fccl-interface")

(defclass fccl-publisher-interface ()
  ((command-pub :reader command-pub :initarg :command-pub)
   (config-pub :reader config-pub :initarg :config-pub)
   (state-sub :reader state-sub :initarg :state-sub)
   (state-fluent-queue :reader state-fluent-queue :initarg :state-fluent-queue)
   (controller-id :reader controller-id :initarg :controller-id)))

(defun init-cram-fccl ()
  "Init function for fccl-controller-interfaces. Needs to be called before calling any of the other functionality."
  (setf *fccl-controllers* (make-hash-table :test #'equal)))

(defun check-fccl-initialized ()
  "Checks whether init-cram-fccl has already been called. If not, an error will be thrown."
  (unless *fccl-controllers*
    (error
     'simple-error
     :format-control "Called cram-fccl functionality without prior calling of init-cram-fccl."
     :format-argument nil)))

(defun add-fccl-controller-interface (config-topic command-topic state-topic controller-name)
  "Create a new fccl-interface with the given information. The interface will be returned and saved internally. 'controller-name' shall save as unique identifier of the interface."
  (declare (type string config-topic command-topic state-topic controller-name))
  (check-fccl-initialized)
  (when (roslisp-utils:hash-table-has-key *fccl-controllers* controller-name)
    (error
     'simple-error
     :format-control "Tried to add fccl-interface with name that already exists: ~a."
     :format-argument (list controller-name)))
  (let ((new-interface
          (make-instance
           'fccl-publisher-interface
           :command-pub (roslisp:advertise command-topic 
                                           "constraints_msgs/ConstraintCommand")
           :config-pub (roslisp:advertise config-topic 
                                          "constraint_msgs/ConstraintConfig" 
                                          :latch t)
           :state-sub (roslisp:subscribe state-topic 
                                         "constraint_msgs/ConstraintState" 
                                         #'constraints-state-callback)
           :state-fluent-queue (cram-language:make-fluent 
                                :name controller-name :allow-tracing nil)
           :controller-id controller-name)))
    (setf (gethash controller-name *fccl-controllers*) new-interface)
    new-interface))

(defun get-fccl-controller-interface (controller-name)
  "Retrieves fccl-interface associated with 'controller-name'."
  (declare (type string controller-name))
  (check-fccl-initialized)
  (multiple-value-bind (value value-p)
      (gethash controller-name *fccl-controllers*)
    (declare (ignore value-p))
    value))

(defun send-constraints-config (constraints fccl-interface)
  "Takes a list of 'constraints' and sends the resulting configuration-msg via the given 'fccl-interface'"
  (declare (type list constraints)
           (type fccl-publisher-interface fccl-interface))
  (check-fccl-initialized)
  (roslisp:publish (config-pub fccl-interface)
                   (feature-constraints->config-msg constraints
                                                    (controller-id fccl-interface))))

(defun send-constraints-command (constraints fccl-interface)
  "Takes a list of 'constraints' and sends the resulting command-msg via the given 'fccl-interface'"
  (declare (type list constraints)
           (type fccl-publisher-interface fccl-interface))
  (check-fccl-initialized)
  (roslisp:publish (command-pub fccl-interface)
                   (feature-constraints->command-msg constraints
                                                     (controller-id fccl-interface))))

(defun execute-constraints-motion (constraints fccl-interface)
  "Takes a set of 'constraints' and configures and starts a feature constraints controller using its 'fccl-interface'."
  (declare (type list constraints)
           (type fccl-publisher-interface fccl-interface))
  (check-fccl-initialized)
  (send-constraints-config constraints fccl-interface)
  (send-constraints-command constraints fccl-interface))

(defun constraints-state-callback (controller-state-msg)
  "Generic state callback function for fccl-controllers. It tries to identify the corresponding fccl-interface and pushes the state message into the corresponding fluent of the interface."
  (check-fccl-initialized)
  ;; try identifying the fccl-interface by the name provided from the controller
  (roslisp:with-fields (name) controller-state-msg
    (let ((fccl-interface
            (get-fccl-controller-interface name)))
      ;; propagate the state-message into the fluent of our fccl-interface
      ;; TODO(Georg): create a class to hold this state information in cram_feature_constraints.
      ;;              this will allow abstracting away constraint_msgs for depending packages...
      (push controller-state-msg (cram-language:value (state-fluent-queue fccl-interface))))))

(defun get-constraints-finished-fluent (fccl-interface)
  "Returns the fluent which signals finished motion execution."
  ;; TODO(@Georg): implement me!
  (declare (ignore fccl-interface))
  (error
     'simple-error
     :format-control "Get-constraints-finished-fluent in packge cram_fccl has not yet been implemented."
     :format-argument nil))

(defun get-constraints-state-fluent (fccl-interface)
  "Returns the fluent holding the state feedback from the controller."
  (declare (type fccl-publisher-interface fccl-interface))
  (check-fccl-initialized)
  (state-fluent-queue fccl-interface))