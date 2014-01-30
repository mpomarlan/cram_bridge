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

(in-package :cram-ik-proxy)

(define-condition ik-query-error (error)
  ((text :initarg :text :reader text))
  (:documentation "Condition signalling an error querying an IK service."))

(defun make-ik-proxy-interface (service-namespace)
  "Creates and returns an instance of type `ik-proxy-interface' which is allows querying an
 IK service for inverse kinematics solutions."
  (declare (type string service-namespace))
  (multiple-value-bind (root-link tip-link joint-names) (get-solver-info service-namespace)
    (let ((service-client (make-instance 
                           'roslisp:persistent-service
                           :service-name (concatenate 'string service-namespace "/get_ik")
                           :service-type "iai_kinematics_msgs/GetPositionIK")))
      (make-instance 'ik-proxy-interface 
                     :ik-client service-client
                     :ik-root-link root-link
                     :ik-tip-link tip-link
                     :joint-names joint-names))))

(defun cleanup-ik-proxy-interface (interface)
  "Cleans up the internals of ik-proxy-interface `interface'."
  (declare (type ik-proxy-interface interface))
  (roslisp:close-persistent-service (ik-client interface)))

(defgeneric get-ik (interface goal seed-state)
  (:documentation "Queries `interface' for an IK solution for `goal' using `seed-state' as
 a starting point of the solver."))

(defmethod get-ik ((interface ik-proxy-interface) (goal-pose cl-tf:pose-stamped)
                   (seed-state list))
  "Queries the server behind `interface' for an IK solution around `seed-state' putting the
 ik-root-link at `goal-pose'."
  (unless (eql (length seed-state) (length (joint-names interface)))
    (error 'ik-query-error :text "Given seed-state has not the right amount of values."))
  (roslisp:with-fields ((solution (joint_state solution))
                        (error-code (val error_code)))
      (roslisp:call-persistent-service
       (ik-client interface)
       :ik_request (make-ik-request interface goal-pose seed-state)
       :timeout 1.0)
    (unless (eql error-code (roslisp-msg-protocol:symbol-code
                           'iai_kinematics_msgs-msg:ErrorCodes :success))
      (error 'ik-query-error :text "IK Solver returned with no solution, of course."))
    solution))